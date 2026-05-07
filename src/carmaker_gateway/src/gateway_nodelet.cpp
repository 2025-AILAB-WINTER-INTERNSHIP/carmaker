#include <carmaker_gateway/gateway_nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Header.h>
namespace carmaker_gateway {

/**
 * @brief Zero-copy conversion from boost::shared_ptr (ROS 1) to std::shared_ptr (Core).
 * Uses aliasing constructor to keep the original boost pointer alive.
 */
template<typename T>
std::shared_ptr<const T> toStdPtr(const boost::shared_ptr<const T>& p) {
    return std::shared_ptr<const T>(p.get(), [p](const T*){});
}

void GatewayNodelet::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    // Load synchronization and fault-tolerance parameters
    pnh_.param("use_anchor_trigger", use_anchor_trigger_, true); // Default to Event-driven (100Hz)
    pnh_.param("sync_rate_hz", sync_rate_hz_, 30.0);
    pnh_.param("time_slop_ms", time_slop_ms_, 15.0);
    time_slop_sec_ = time_slop_ms_ / 1000.0;
    pnh_.param("enable_virtual_heartbeat", enable_virtual_heartbeat_, true);
    pnh_.param("max_heartbeat_cycles", max_heartbeat_cycles_, -1);
    pnh_.param("watchdog_timeout", watchdog_timeout_, 100.0);
    pnh_.param("watchdog_ratio", watchdog_ratio_, 1.5);
    pnh_.param("qos_reliability", qos_reliability_, std::string("BEST_EFFORT"));
    pnh_.param("anchor_topic", anchor_topic_, std::string("/carmaker/dynamic_info"));
    pnh_.param("output_topic", output_topic_, std::string("/gateway/synced_data"));
    pnh_.param("status_topic", status_topic_, std::string("/gateway/status"));
    pnh_.param("time_jump_threshold", time_jump_threshold_, 1.0);
    pnh_.param("diag_hit_rate_warn", diag_hit_rate_warn_, 0.5);

    if (!pnh_.getParam("input_topics", input_topics_)) {
        NODELET_WARN("No input_topics specified, using default perception topics.");
        input_topics_ = {"/mono/fisheye/front/image_raw", "/carmaker/objects"};
    }

    // Determine QoS transport hints based on configuration
    ros::TransportHints hints;
    if (qos_reliability_ == "BEST_EFFORT") {
        hints.udp();
    } else {
        hints.tcp().tcpNoDelay();
    }

    // Initialize Subscribers
    anchor_sub_ = nh_.subscribe(anchor_topic_, 10, &GatewayNodelet::DynamicsInfoCallback, this, hints);

    for (const auto& topic : input_topics_) {
        NODELET_INFO("Registering input topic: %s", topic.c_str());
        if (topic.find("image_raw") != std::string::npos) {
            auto& ctx = camera_contexts_[topic];
            ctx = std::make_unique<CameraContext>();
            ctx->name = topic;

            auto img_cache = ctx->image_cache;
            ctx->image_sub = nh_.subscribe<sensor_msgs::Image>(topic, 2,
                [this, img_cache](const sensor_msgs::Image::ConstPtr& msg) {
                    img_cache->Push(toStdPtr(msg));
                }, ros::VoidPtr(), hints);

            std::string info_topic = topic;
            size_t pos = info_topic.find("image_raw");
            if (pos != std::string::npos) {
                info_topic.replace(pos, 9, "camera_info");
                auto info_cache = ctx->info_cache;
                ctx->info_sub = nh_.subscribe<sensor_msgs::CameraInfo>(info_topic, 2,
                    [this, info_cache](const sensor_msgs::CameraInfo::ConstPtr& msg) {
                        info_cache->Push(toStdPtr(msg));
                    }, ros::VoidPtr(), hints);
            }
        } else if (topic.find("objects") != std::string::npos) {
            objects_sub_ = nh_.subscribe(topic, 10, &GatewayNodelet::objectsCallback, this, hints);
        } else if (topic.find("uaq_out") != std::string::npos) {
            uaq_sub_ = nh_.subscribe(topic, 10, &GatewayNodelet::uaqCallback, this, hints);
        }
    }

    // Populate fast-access list for hot-path iteration
    for (auto& kv : camera_contexts_) {
        fast_camera_list_.push_back(kv.second.get());
    }

    synced_pub_ = nh_.advertise<carmaker_msgs::SyncedData>(output_topic_, 10);
    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(status_topic_, 1);

    // Trigger Mode: Event-driven (Anchor) vs Periodic (Timer)
    sync_timer_ = nh_.createTimer(ros::Duration(1.0 / sync_rate_hz_), &GatewayNodelet::timerCallback, this);
    diag_timer_ = nh_.createTimer(ros::Duration(1.0), &GatewayNodelet::diagnosticsCallback, this);

    if (use_anchor_trigger_) {
        NODELET_INFO("Gateway Mode: Event-Driven (Timer active as Watchdog)");
    } else {
        NODELET_INFO("Gateway Mode: Periodic Timer (%.1f Hz)", sync_rate_hz_);
    }

    cached_sync_targets_ = num_sync_targets_();
}

void GatewayNodelet::DynamicsInfoCallback(const carmaker_msgs::DynamicsInfo::ConstPtr& msg) {
    last_anchor_arrival_time_.store(ros::WallTime::now().toSec(), std::memory_order_relaxed);
    auto std_msg = toStdPtr(msg);
    anchor_cache_.Push(std_msg);

    // [EVENT-DRIVEN] Process immediately on anchor arrival
    if (use_anchor_trigger_) {
        processSynchronization(std_msg->header.stamp.toSec(), std_msg);
    }
}

void GatewayNodelet::objectsCallback(const carmaker_msgs::Objects::ConstPtr& msg) {
    objects_cache_.Push(toStdPtr(msg));
}

void GatewayNodelet::uaqCallback(const carmaker_msgs::UAQ_Out::ConstPtr& msg) {
    uaq_cache_.Push(toStdPtr(msg));
}

void GatewayNodelet::timerCallback(const ros::TimerEvent& event) {
    // [WATCHDOG] If event-driven and anchor is recently arrived, skip timer execution
    if (use_anchor_trigger_) {
        double now = ros::WallTime::now().toSec();
        if ((now - last_anchor_arrival_time_.load(std::memory_order_relaxed)) < (watchdog_ratio_ / sync_rate_hz_)) {
            return;
        }
    }

    double target_time = 0.0;
    bool is_valid = false;

    auto anchor = anchor_cache_.GetMostRecent();
    if (anchor) {
        target_time = anchor->header.stamp.toSec();
        double now = ros::Time::now().toSec();
        // Hybrid Watchdog: Use header.stamp for SimTime mode, or time field for WallTime mode
        double anchor_ref_time = ros::Time::isSimTime() ? anchor->header.stamp.toSec() : anchor->time.toSec();

        if (std::abs(anchor_ref_time - now) <= (watchdog_timeout_ / 1000.0)) {
            last_valid_anchor_time_.store(target_time, std::memory_order_relaxed);
            base_virtual_time_ = target_time; // Update sync base
            heartbeat_counter_.store(0, std::memory_order_relaxed);
            is_valid = true;
        }
    }

    if (!is_valid && enable_virtual_heartbeat_) {
        heartbeat_counter_.fetch_add(1, std::memory_order_relaxed);
        if (max_heartbeat_cycles_ == -1 || heartbeat_counter_.load(std::memory_order_relaxed) <= max_heartbeat_cycles_) {
            // [TIME SYNC] Using multiplication to prevent floating-point drift
            target_time = base_virtual_time_ + (static_cast<double>(heartbeat_counter_.load(std::memory_order_relaxed)) / sync_rate_hz_);
            last_valid_anchor_time_.store(target_time, std::memory_order_relaxed);
            anchor_cache_.NotifyVirtualHeartbeatTriggered();
            processSynchronization(target_time, nullptr);
        }
    } else if (is_valid && !use_anchor_trigger_) {
        // Only publish from timer if NOT in event-driven mode
        processSynchronization(target_time, anchor);
    }
}

void GatewayNodelet::processSynchronization(double target_time, const std::shared_ptr<const carmaker_msgs::DynamicsInfo>& anchor) {
    // 1. Thread-safe Message Allocation (Avoids race condition with async subscribers)
    auto out_msg = boost::make_shared<carmaker_msgs::SyncedData>();
    out_msg->sensors.reserve(3); // Anchor(Dynamics) + Objects + UAQ
    out_msg->cameras.reserve(fast_camera_list_.size());
    out_msg->header.stamp = ros::Time(target_time);
    out_msg->is_valid = (anchor != nullptr);

    total_sync_attempts_.fetch_add(1, std::memory_order_relaxed);

    // [SENSOR FUSION HELPER] Generic lambda for unified matching logic
    // is_anchor: If true, excludes this sensor from 'hits' count (it's the reference)
    auto sync_sensor = [&](const std::string& name, auto match_ptr, auto& out_field, bool is_anchor = false) {
        carmaker_msgs::SensorStatus status;
        status.name = name;
        if (match_ptr && std::abs(match_ptr->header.stamp.toSec() - target_time) <= time_slop_sec_) {
            out_field = *match_ptr;
            status.status = true;
            if (!is_anchor) cache_hits_.fetch_add(1, std::memory_order_relaxed);
        } else {
            status.status = false;
            if (match_ptr && !is_anchor) slop_violations_.fetch_add(1, std::memory_order_relaxed);
        }
        out_msg->sensors.push_back(std::move(status));
    };

    // 2. Perform Fusion across all modes
    sync_sensor("dynamics", anchor, out_msg->dynamic_info, true);
    sync_sensor("objects", objects_cache_.GetBestMatch(target_time), out_msg->objects);
    sync_sensor("uaq", uaq_cache_.GetBestMatch(target_time), out_msg->uaq_out);

    // [CAMERA FUSION HELPER] Specialized lambda for camera matching and sparse payload logic
    auto sync_camera = [&](CameraContext* ctx) {
        auto img = ctx->image_cache->GetBestMatch(target_time);
        auto info = ctx->info_cache->GetBestMatch(target_time);

        carmaker_msgs::CameraData cam_data;
        cam_data.name = ctx->name;

        if (img && std::abs(img->header.stamp.toSec() - target_time) <= time_slop_sec_) {
            cache_hits_.fetch_add(1, std::memory_order_relaxed);
            double current_stamp = img->header.stamp.toSec();
            double last_stamp = ctx->last_published_stamp.load(std::memory_order_relaxed);

            shallowCopyImageMetadata(*img, cam_data.image);

            if (info && std::abs(info->header.stamp.toSec() - target_time) <= time_slop_sec_) {
                cam_data.info = *info;
                cache_hits_.fetch_add(1, std::memory_order_relaxed);
            } else if (info) {
                slop_violations_.fetch_add(1, std::memory_order_relaxed);
            }

            // [SMART SPARSE PAYLOAD]
            // Keep status=true to indicate sync success, but only send data for NEW frames to save bandwidth.
            bool is_new_frame = (current_stamp > last_stamp) ||
                                (current_stamp < (last_stamp - time_jump_threshold_));

            if (is_new_frame) {
                cam_data.image.data = img->data;
                ctx->last_published_stamp.store(current_stamp, std::memory_order_relaxed);
            }

            cam_data.status = true;

        } else {
            cam_data.status = false;
            if (img) slop_violations_.fetch_add(1, std::memory_order_relaxed);
        }
        out_msg->cameras.push_back(std::move(cam_data));
    };

    for (auto* ctx : fast_camera_list_) {
        sync_camera(ctx);
    }

    synced_pub_.publish(out_msg);
}

void GatewayNodelet::diagnosticsCallback(const ros::TimerEvent& event) {
    diagnostic_msgs::DiagnosticArray diag_array;
    diag_array.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus status;
    status.name = "carmaker_gateway: Synchronization Performance";
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "Running";
    status.hardware_id = ros::this_node::getName();

    uint64_t total = total_sync_attempts_.exchange(0, std::memory_order_relaxed);
    uint64_t hits = cache_hits_.exchange(0, std::memory_order_relaxed);
    uint64_t slops = slop_violations_.exchange(0, std::memory_order_relaxed);
    uint64_t drops = anchor_cache_.GetAndResetDropCount();

    auto add_kv = [&](const std::string& key, const std::string& value) {
        diagnostic_msgs::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
    };

    add_kv("Sync Attempts (Hz)", std::to_string(total));
    add_kv("Cache Hits", std::to_string(hits));
    add_kv("Slop Violations", std::to_string(slops));
    add_kv("Anchor Drops", std::to_string(drops));
    add_kv("Virtual Heartbeat Active", (heartbeat_counter_.load(std::memory_order_relaxed) > 0 ? "Yes" : "No"));

    if (total > 0 && cached_sync_targets_ > 0) {
        double hit_rate = static_cast<double>(hits) / (total * cached_sync_targets_);
        double slop_rate = static_cast<double>(slops) / (total * cached_sync_targets_);

        add_kv("Hit Rate (Approx)", std::to_string(hit_rate * 100.0) + "%");
        add_kv("Slop Violation Rate", std::to_string(slop_rate * 100.0) + "%");

        if (hit_rate < diag_hit_rate_warn_) {
            status.level = diagnostic_msgs::DiagnosticStatus::WARN;
            // Refined message based on violation type
            if (slop_rate > 0.2) {
                status.message = "Low Sync Hit Rate: High Slop Violations (Check latency)";
            } else {
                status.message = "Low Sync Hit Rate: Significant Data Loss Detected";
            }
        }
    } else if (total > 0 && cached_sync_targets_ == 0) {
        add_kv("Hit Rate", "N/A (Anchor Only)");
    }

    diag_array.status.push_back(status);
    diag_pub_.publish(diag_array);
}

int GatewayNodelet::num_sync_targets_() const {
    int count = 0; // count only the sensors to be synchronized, excluding the anchor
    if (objects_sub_) count++;
    if (uaq_sub_) count++;
    for (const auto& kv : camera_contexts_) {
        count++; // Image
        if (kv.second->info_sub) count++; // CameraInfo
    }
    return count;
}

void GatewayNodelet::shallowCopyImageMetadata(const sensor_msgs::Image& src, sensor_msgs::Image& dst) {
    dst.header = src.header;
    dst.height = src.height;
    dst.width = src.width;
    dst.encoding = src.encoding;
    dst.is_bigendian = src.is_bigendian;
    dst.step = src.step;
    // data field is intentionally not copied to save bandwidth on stale frames
}

} // namespace carmaker_gateway

PLUGINLIB_EXPORT_CLASS(carmaker_gateway::GatewayNodelet, nodelet::Nodelet)
