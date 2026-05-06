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
    pnh_.param("sync_rate_hz", sync_rate_hz_, 30.0);
    pnh_.param("time_slop_ms", time_slop_ms_, 15.0);
    pnh_.param("enable_virtual_heartbeat", enable_virtual_heartbeat_, true);
    pnh_.param("max_heartbeat_cycles", max_heartbeat_cycles_, -1);
    pnh_.param("watchdog_timeout", watchdog_timeout_, 100.0);
    pnh_.param("anchor_topic", anchor_topic_, std::string("/carmaker/dynamic_info"));

    if (!pnh_.getParam("input_topics", input_topics_)) {
        NODELET_WARN("No input_topics specified, using default perception topics.");
        input_topics_ = {"/mono/fisheye/front/image_raw", "/carmaker/objects"};
    }

    // Initialize Subscribers
    anchor_sub_ = nh_.subscribe(anchor_topic_, 10, &GatewayNodelet::DynamicsInfoCallback, this);

    for (const auto& topic : input_topics_) {
        NODELET_INFO("Registering input topic: %s", topic.c_str());
        if (topic.find("image_raw") != std::string::npos) {
            camera_image_caches_[topic] = std::make_shared<carmaker_gateway::LockFreeTimeRingBuffer<sensor_msgs::Image, 5, carmaker_gateway::TimestampExtractor<sensor_msgs::Image>>>();
            camera_image_subs_.push_back(nh_.subscribe<sensor_msgs::Image>(topic, 2, boost::bind(&GatewayNodelet::cameraImageCallback, this, _1, topic)));
        } else if (topic.find("camera_info") != std::string::npos) {
            camera_info_caches_[topic] = std::make_shared<carmaker_gateway::LockFreeTimeRingBuffer<sensor_msgs::CameraInfo, 5, carmaker_gateway::TimestampExtractor<sensor_msgs::CameraInfo>>>();
            camera_info_subs_.push_back(nh_.subscribe<sensor_msgs::CameraInfo>(topic, 2, boost::bind(&GatewayNodelet::cameraInfoCallback, this, _1, topic)));
        } else if (topic.find("objects") != std::string::npos) {
            objects_sub_ = nh_.subscribe(topic, 10, &GatewayNodelet::objectsCallback, this);
        } else if (topic.find("uaq_out") != std::string::npos) {
            uaq_sub_ = nh_.subscribe(topic, 10, &GatewayNodelet::uaqCallback, this);
        }
    }

    synced_pub_ = nh_.advertise<carmaker_msgs::SyncedData>("/gateway/synced_data", 10);

    // Pre-allocate message to prevent heap fragmentation
    out_msg_ = boost::make_shared<carmaker_msgs::SyncedData>();
    out_msg_->sensors.reserve(3); // dynamics, objects, uaq
    out_msg_->cameras.reserve(4);

    // Fixed-rate synchronization timer
    sync_timer_ = nh_.createTimer(ros::Duration(1.0 / sync_rate_hz_), &GatewayNodelet::timerCallback, this);

    NODELET_INFO("Gateway Nodelet initialized. Deterministic time-sweep active. Primary anchor: %s", anchor_topic_.c_str());
}

void GatewayNodelet::DynamicsInfoCallback(const carmaker_msgs::DynamicsInfo::ConstPtr& msg) {
    anchor_cache_.Push(toStdPtr(msg));
}

void GatewayNodelet::objectsCallback(const carmaker_msgs::Objects::ConstPtr& msg) {
    objects_cache_.Push(toStdPtr(msg));
}

void GatewayNodelet::uaqCallback(const carmaker_msgs::UAQ_Out::ConstPtr& msg) {
    uaq_cache_.Push(toStdPtr(msg));
}

void GatewayNodelet::cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic_name) {
    auto it = camera_image_caches_.find(topic_name);
    if (it != camera_image_caches_.end()) {
        it->second->Push(toStdPtr(msg));
    }
}

void GatewayNodelet::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, const std::string& topic_name) {
    auto it = camera_info_caches_.find(topic_name);
    if (it != camera_info_caches_.end()) {
        it->second->Push(toStdPtr(msg));
    }
}

void GatewayNodelet::timerCallback(const ros::TimerEvent& event) {
    double target_time = 0.0;
    bool is_valid = true;

    // 1. Anchor check (Deterministic clock reference)
    auto anchor = anchor_cache_.GetMostRecent();

    if (anchor) {
        target_time = anchor->time.toSec(); // Use SimTime for logic
        double wall_now = ros::WallTime::now().toSec();
        double anchor_wall = anchor->header.stamp.toSec(); // header.stamp is WallTime

        // Watchdog: Check if anchor's WallTime is within timeout of current system WallTime
        if (std::abs(anchor_wall - wall_now) <= (watchdog_timeout_ / 1000.0)) {
            last_valid_anchor_time_ = target_time;
            heartbeat_counter_ = 0;
            is_valid = true;
        } else {
            is_valid = false;
        }
    }

    if (!is_valid) {
        // 2. Extrapolation: Calculate target time based on system cycle
        if (enable_virtual_heartbeat_) {
            heartbeat_counter_++;
            if (max_heartbeat_cycles_ != -1 && heartbeat_counter_ > max_heartbeat_cycles_) {
                NODELET_ERROR_THROTTLE(1.0, "Enterprise Gateway: System stalled. Stale anchor.");
                return;
            }
            target_time = last_valid_anchor_time_ + (1.0 / sync_rate_hz_);
            last_valid_anchor_time_ = target_time;
            anchor_cache_.NotifyVirtualHeartbeatTriggered();
            anchor = nullptr;
        } else {
            return;
        }
    }

    // 3. Clear and Reset pre-allocated message
    out_msg_->sensors.clear();
    out_msg_->cameras.clear();
    out_msg_->header.stamp = ros::Time(target_time);
    out_msg_->is_valid = is_valid;

    // 4. Deterministic Time Sweep (Lookback across all sensor history)
    carmaker_msgs::SensorStatus dyn_status;
    dyn_status.name = "dynamics";
    if (anchor) {
        out_msg_->dynamic_info = *anchor;
        dyn_status.status = true;
    } else {
        dyn_status.status = false;
    }
    out_msg_->sensors.push_back(dyn_status);

    // Objects Match
    auto objects = objects_cache_.GetBestMatch(target_time);
    carmaker_msgs::SensorStatus obj_status;
    obj_status.name = "objects";
    if (objects && std::abs(objects->time.toSec() - target_time) <= (time_slop_ms_ / 1000.0)) {
        out_msg_->objects = *objects;
        obj_status.status = true;
    } else {
        obj_status.status = false;
    }
    out_msg_->sensors.push_back(obj_status);

    // UAQ Match
    auto uaq = uaq_cache_.GetBestMatch(target_time);
    carmaker_msgs::SensorStatus uaq_status;
    uaq_status.name = "uaq";
    if (uaq) {
        double slop = std::abs(uaq->time.toSec() - target_time);
        if (slop <= (time_slop_ms_ / 1000.0)) {
            out_msg_->uaq_out = *uaq;
            uaq_status.status = true;
        } else {
            uaq_status.status = false;
        }
    } else {
        uaq_status.status = false;
    }
    out_msg_->sensors.push_back(uaq_status);

    for (const auto& kv : camera_image_caches_) {
        const std::string& img_topic = kv.first;
        auto img = kv.second->GetBestMatch(target_time);

        carmaker_msgs::CameraData cam_data;
        cam_data.name = img_topic;

        // Prepare corresponding CameraInfo topic name
        std::string info_topic = img_topic;
        size_t pos = info_topic.find("image_raw");
        if (pos != std::string::npos) {
            info_topic.replace(pos, 9, "camera_info");
        }

        if (img && std::abs(img->header.stamp.toSec() - target_time) <= (time_slop_ms_ / 1000.0)) {
            cam_data.image = *img;

            auto it_info = camera_info_caches_.find(info_topic);
            if (it_info != camera_info_caches_.end()) {
                auto info = it_info->second->GetBestMatch(target_time);
                if (info && std::abs(info->header.stamp.toSec() - target_time) <= (time_slop_ms_ / 1000.0)) {
                    cam_data.info = *info;
                } else {
                    cam_data.info = sensor_msgs::CameraInfo();
                }
            } else {
                cam_data.info = sensor_msgs::CameraInfo();
            }
            cam_data.status = true;
        } else {
            // PADDING: Maintain fixed index even on failure
            cam_data.image = sensor_msgs::Image();
            cam_data.info = sensor_msgs::CameraInfo();
            cam_data.status = false;
        }
        out_msg_->cameras.push_back(cam_data);
    }

    synced_pub_.publish(out_msg_);
}

} // namespace carmaker_gateway

PLUGINLIB_EXPORT_CLASS(carmaker_gateway::GatewayNodelet, nodelet::Nodelet)
