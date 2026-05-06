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
    anchor_sub_ = nh_.subscribe(anchor_topic_, 10, &GatewayNodelet::dynamicInfoCallback, this);

    for (const auto& topic : input_topics_) {
        if (topic.find("image_raw") != std::string::npos) {
            image_caches_[topic] = std::make_shared<carmaker_gateway::LockFreeSensorCache<sensor_msgs::Image>>();
            // boost::bind to pass topic name to callback
            image_subs_.push_back(nh_.subscribe<sensor_msgs::Image>(topic, 2, boost::bind(&GatewayNodelet::imageCallback, this, _1, topic)));
        } else if (topic.find("objects") != std::string::npos) {
            objects_sub_ = nh_.subscribe(topic, 10, &GatewayNodelet::objectsCallback, this);
        } else if (topic.find("uaq_out") != std::string::npos) {
            uaq_sub_ = nh_.subscribe(topic, 10, &GatewayNodelet::uaqCallback, this);
        }
    }

    synced_pub_ = nh_.advertise<carmaker_msgs::SyncedData>("/gateway/synced_data", 10);

    // Fixed-rate synchronization timer
    sync_timer_ = nh_.createTimer(ros::Duration(1.0 / sync_rate_hz_), &GatewayNodelet::timerCallback, this);

    NODELET_INFO("Gateway Nodelet initialized. Primary anchor: %s", anchor_topic_.c_str());
}

void GatewayNodelet::dynamicInfoCallback(const carmaker_msgs::DynamicInfo::ConstPtr& msg) {
    anchor_cache_.Push(toStdPtr(msg));
}

void GatewayNodelet::objectsCallback(const carmaker_msgs::Objects::ConstPtr& msg) {
    objects_cache_.Update(toStdPtr(msg));
}

void GatewayNodelet::uaqCallback(const carmaker_msgs::UAQ_Out::ConstPtr& msg) {
    uaq_cache_.Update(toStdPtr(msg));
}

void GatewayNodelet::imageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic_name) {
    auto it = image_caches_.find(topic_name);
    if (it != image_caches_.end()) {
        it->second->Update(toStdPtr(msg));
    }
}

void GatewayNodelet::timerCallback(const ros::TimerEvent& event) {
    double now = ros::Time::now().toSec();

    // Temporal matching using the anchor cache
    auto anchor = anchor_cache_.GetBestMatch(now);

    // Validate anchor freshness against the allowed slop
    if (anchor && std::abs(anchor->header.stamp.toSec() - now) > (time_slop_ms_ / 1000.0)) {
        anchor = nullptr;
    }

    bool is_valid = true;

    // Handle missing anchor topic
    if (!anchor) {
        is_valid = false;
        if (enable_virtual_heartbeat_) {
            heartbeat_counter_++;
            if (max_heartbeat_cycles_ != -1 && heartbeat_counter_ > max_heartbeat_cycles_) {
                NODELET_ERROR_THROTTLE(1.0, "Gateway: Maximum heartbeat cycles exceeded. System stalled.");
            } else {
                NODELET_WARN_THROTTLE(2.0, "Gateway: Anchor missing. Operating in virtual heartbeat mode.");
                anchor_cache_.NotifyVirtualHeartbeatTriggered();
            }
        }
    } else {
        heartbeat_counter_ = 0;
    }

    // Assemble the synchronized data packet
    carmaker_msgs::SyncedDataPtr out_msg = boost::make_shared<carmaker_msgs::SyncedData>();
    out_msg->header.stamp = ros::Time::now();
    out_msg->is_valid = is_valid;

    if (anchor) {
        out_msg->dynamic_info = *anchor;
    }

    // Populate objects and UAQ with latest available data
    auto objects = objects_cache_.GetLatest();
    if (objects) {
        out_msg->objects = *objects;
        out_msg->sensor_names.push_back("objects");
        out_msg->sensor_status.push_back(true);
    } else {
        out_msg->sensor_names.push_back("objects");
        out_msg->sensor_status.push_back(false);
    }

    auto uaq = uaq_cache_.GetLatest();
    if (uaq) {
        out_msg->uaq_out = *uaq;
        out_msg->sensor_names.push_back("uaq");
        out_msg->sensor_status.push_back(true);
    } else {
        out_msg->sensor_names.push_back("uaq");
        out_msg->sensor_status.push_back(false);
    }

    // Populate multi-camera payload
    for (const auto& kv : image_caches_) {
        auto img = kv.second->GetLatest();
        out_msg->sensor_names.push_back(kv.first);
        if (img) {
            out_msg->images.push_back(*img);
            out_msg->sensor_status.push_back(true);
        } else {
            out_msg->sensor_status.push_back(false);
        }
    }

    synced_pub_.publish(out_msg);
}

} // namespace carmaker_gateway

PLUGINLIB_EXPORT_CLASS(carmaker_gateway::GatewayNodelet, nodelet::Nodelet)
