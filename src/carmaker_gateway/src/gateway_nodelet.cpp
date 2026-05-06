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
        if (topic.find("image_raw") != std::string::npos) {
            image_caches_[topic] = std::make_shared<carmaker_gateway::LockFreeTimeRingBuffer<sensor_msgs::Image>>();
            image_subs_.push_back(nh_.subscribe<sensor_msgs::Image>(topic, 2, boost::bind(&GatewayNodelet::imageCallback, this, _1, topic)));
        } else if (topic.find("objects") != std::string::npos) {
            objects_sub_ = nh_.subscribe(topic, 10, &GatewayNodelet::objectsCallback, this);
        } else if (topic.find("uaq_out") != std::string::npos) {
            uaq_sub_ = nh_.subscribe(topic, 10, &GatewayNodelet::uaqCallback, this);
        }
    }

    synced_pub_ = nh_.advertise<carmaker_msgs::SyncedData>("/gateway/synced_data", 10);

    // Pre-allocate message to prevent heap fragmentation
    out_msg_ = boost::make_shared<carmaker_msgs::SyncedData>();
    out_msg_->sensor_names.reserve(input_topics_.size() + 2);
    out_msg_->sensor_status.reserve(input_topics_.size() + 2);
    out_msg_->images.reserve(4);

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

void GatewayNodelet::imageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic_name) {
    auto it = image_caches_.find(topic_name);
    if (it != image_caches_.end()) {
        it->second->Push(toStdPtr(msg));
    }
}

void GatewayNodelet::timerCallback(const ros::TimerEvent& event) {
    double target_time = 0.0;
    bool is_valid = true;

    // 1. Anchor check (Deterministic clock reference)
    auto anchor = anchor_cache_.GetMostRecent();
    double wall_now = ros::Time::now().toSec();

    if (anchor && std::abs(anchor->header.stamp.toSec() - wall_now) <= (watchdog_timeout_ / 1000.0)) {
        // Normal operation: Use anchor's own timestamp for synchronization
        target_time = anchor->header.stamp.toSec();
        last_valid_anchor_time_ = target_time;
        heartbeat_counter_ = 0;
    } else {
        // 2. Extrapolation: Calculate target time based on system cycle
        is_valid = false;
        if (enable_virtual_heartbeat_) {
            heartbeat_counter_++;
            if (max_heartbeat_cycles_ != -1 && heartbeat_counter_ > max_heartbeat_cycles_) {
                NODELET_ERROR_THROTTLE(1.0, "Enterprise Gateway: System stalled. Stale anchor.");
                return;
            }
            // Extrapolate target time (last_time + dt)
            target_time = last_valid_anchor_time_ + (1.0 / sync_rate_hz_);
            last_valid_anchor_time_ = target_time;
            anchor_cache_.NotifyVirtualHeartbeatTriggered();
            anchor = nullptr;
        } else {
            return;
        }
    }

    // 3. Clear and Reset pre-allocated message
    out_msg_->sensor_names.clear();
    out_msg_->sensor_status.clear();
    out_msg_->images.clear();
    out_msg_->header.stamp = ros::Time(target_time);
    out_msg_->is_valid = is_valid;

    // 4. Deterministic Time Sweep (Lookback across all sensor history)
    out_msg_->sensor_names.push_back("dynamics");
    if (anchor) {
        out_msg_->dynamic_info = *anchor;
        out_msg_->sensor_status.push_back(true);
    } else {
        out_msg_->sensor_status.push_back(false);
    }

    // Objects Match
    auto objects = objects_cache_.GetBestMatch(target_time);
    out_msg_->sensor_names.push_back("objects");
    if (objects && std::abs(objects->header.stamp.toSec() - target_time) <= (time_slop_ms_ / 1000.0)) {
        out_msg_->objects = *objects;
        out_msg_->sensor_status.push_back(true);
    } else {
        out_msg_->sensor_status.push_back(false);
    }

    // UAQ Match
    auto uaq = uaq_cache_.GetBestMatch(target_time);
    out_msg_->sensor_names.push_back("uaq");
    if (uaq && std::abs(uaq->header.stamp.toSec() - target_time) <= (time_slop_ms_ / 1000.0)) {
        out_msg_->uaq_out = *uaq;
        out_msg_->sensor_status.push_back(true);
    } else {
        out_msg_->sensor_status.push_back(false);
    }

    // Multi-camera Lookback Sweep
    for (const auto& kv : image_caches_) {
        auto img = kv.second->GetBestMatch(target_time);
        out_msg_->sensor_names.push_back(kv.first);
        if (img && std::abs(img->header.stamp.toSec() - target_time) <= (time_slop_ms_ / 1000.0)) {
            out_msg_->images.push_back(*img);
            out_msg_->sensor_status.push_back(true);
        } else {
            out_msg_->sensor_status.push_back(false);
        }
    }

    synced_pub_.publish(out_msg_);
}

} // namespace carmaker_gateway

PLUGINLIB_EXPORT_CLASS(carmaker_gateway::GatewayNodelet, nodelet::Nodelet)
