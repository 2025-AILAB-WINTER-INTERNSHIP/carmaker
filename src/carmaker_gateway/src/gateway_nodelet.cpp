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
    pnh_.param("enable_virtual_heartbeat", enable_virtual_heartbeat_, true);
    pnh_.param("max_heartbeat_cycles", max_heartbeat_cycles_, -1);
    pnh_.param("watchdog_timeout", watchdog_timeout_, 100.0);
    pnh_.param("watchdog_ratio", watchdog_ratio_, 1.5);
    pnh_.param("qos_reliability", qos_reliability_, std::string("BEST_EFFORT"));
    pnh_.param("anchor_topic", anchor_topic_, std::string("/carmaker/dynamic_info"));
    pnh_.param("output_topic", output_topic_, std::string("/gateway/synced_data"));

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

    // Trigger Mode: Event-driven (Anchor) vs Periodic (Timer)
    sync_timer_ = nh_.createTimer(ros::Duration(1.0 / sync_rate_hz_), &GatewayNodelet::timerCallback, this);
    
    if (use_anchor_trigger_) {
        NODELET_INFO("Gateway Mode: Event-Driven (Timer active as Watchdog)");
    } else {
        NODELET_INFO("Gateway Mode: Periodic Timer (%.1f Hz)", sync_rate_hz_);
    }
}

void GatewayNodelet::DynamicsInfoCallback(const carmaker_msgs::DynamicsInfo::ConstPtr& msg) {
    last_anchor_arrival_time_ = ros::WallTime::now().toSec();
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
        if ((now - last_anchor_arrival_time_) < (watchdog_ratio_ / sync_rate_hz_)) {
            return; 
        }
    }

    double target_time = 0.0;
    bool is_valid = false;

    auto anchor = anchor_cache_.GetMostRecent();
    if (anchor) {
        target_time = anchor->header.stamp.toSec();
        double wall_now = ros::WallTime::now().toSec();
        if (std::abs(anchor->time.toSec() - wall_now) <= (watchdog_timeout_ / 1000.0)) {
            last_valid_anchor_time_ = target_time;
            heartbeat_counter_ = 0;
            is_valid = true;
        }
    }

    if (!is_valid && enable_virtual_heartbeat_) {
        heartbeat_counter_++;
        if (max_heartbeat_cycles_ == -1 || heartbeat_counter_ <= max_heartbeat_cycles_) {
            target_time = last_valid_anchor_time_ + (1.0 / sync_rate_hz_);
            last_valid_anchor_time_ = target_time;
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
    out_msg->sensors.reserve(3);
    out_msg->cameras.reserve(fast_camera_list_.size());
    out_msg->header.stamp = ros::Time(target_time);
    out_msg->is_valid = (anchor != nullptr);

    // 2. Dynamics Info
    carmaker_msgs::SensorStatus dyn_status;
    dyn_status.name = "dynamics";
    if (anchor) {
        out_msg->dynamic_info = *anchor;
        dyn_status.status = true;
    } else {
        dyn_status.status = false;
    }
    out_msg->sensors.push_back(dyn_status);

    // 3. Objects Match
    auto objects = objects_cache_.GetBestMatch(target_time);
    carmaker_msgs::SensorStatus obj_status;
    obj_status.name = "objects";
    if (objects && std::abs(objects->header.stamp.toSec() - target_time) <= (time_slop_ms_ / 1000.0)) {
        out_msg->objects = *objects;
        obj_status.status = true;
    } else {
        obj_status.status = false;
    }
    out_msg->sensors.push_back(obj_status);

    // 4. UAQ Match
    auto uaq = uaq_cache_.GetBestMatch(target_time);
    carmaker_msgs::SensorStatus uaq_status;
    uaq_status.name = "uaq";
    if (uaq && std::abs(uaq->header.stamp.toSec() - target_time) <= (time_slop_ms_ / 1000.0)) {
        out_msg->uaq_out = *uaq;
        uaq_status.status = true;
    } else {
        uaq_status.status = false;
    }
    out_msg->sensors.push_back(uaq_status);

    // 5. Sparse Camera Match (High performance O(1) iteration)
    for (auto* ctx : fast_camera_list_) {
        auto img = ctx->image_cache->GetBestMatch(target_time);
        auto info = ctx->info_cache->GetBestMatch(target_time);

        carmaker_msgs::CameraData cam_data;
        cam_data.name = ctx->name;

        if (img && std::abs(img->header.stamp.toSec() - target_time) <= (time_slop_ms_ / 1000.0)) {
            double current_stamp = img->header.stamp.toSec();
            double last_stamp = ctx->last_published_stamp.load(std::memory_order_relaxed);
            
            // [METADATA ALWAYS] Using helper for elegant copy
            shallowCopyImageMetadata(*img, cam_data.image);

            if (info && std::abs(info->header.stamp.toSec() - target_time) <= (time_slop_ms_ / 1000.0)) {
                cam_data.info = *info;
            }

            // [SPARSE PAYLOAD] Thread-safe check with Time-Jump resilience
            // Trigger on progression OR significant backward jump (Sim reset)
            bool is_new_frame = (current_stamp > last_stamp) || (current_stamp < (last_stamp - 1.0));
            
            if (is_new_frame) {
                cam_data.image.data = img->data; 
                cam_data.status = true;          
                ctx->last_published_stamp.store(current_stamp, std::memory_order_relaxed);
            } else {
                cam_data.status = false; 
            }
        } else {
            cam_data.status = false;
        }
        out_msg->cameras.push_back(std::move(cam_data));
    }

    synced_pub_.publish(out_msg);
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
