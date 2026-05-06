#ifndef CARMAKER_GATEWAY_NODELET_H
#define CARMAKER_GATEWAY_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <carmaker_msgs/DynamicsInfo.h>
#include <carmaker_msgs/Objects.h>
#include <carmaker_msgs/UAQ_Out.h>
#include <carmaker_msgs/SyncedData.h>
#include <carmaker_gateway/core/lock_free_sensor_cache.hpp>
#include <carmaker_gateway/core/monotonic_anchor_cache.hpp>

#include <string>
#include <vector>
#include <memory>
#include <map>

namespace carmaker_gateway {

/**
 * @brief ROS Nodelet wrapper for the CarMaker Gateway.
 * Aggregates and synchronizes asynchronous sensor data (Images, Objects, UAQ)
 * into a single unified message based on a high-precision anchor topic.
 */
class GatewayNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();

private:
    /**
     * @brief Periodic timer callback to publish synchronized data.
     */
    void timerCallback(const ros::TimerEvent& event);

    // Topic subscription callbacks
    void DynamicsInfoCallback(const carmaker_msgs::DynamicsInfo::ConstPtr& msg);
    void objectsCallback(const carmaker_msgs::Objects::ConstPtr& msg);
    void uaqCallback(const carmaker_msgs::UAQ_Out::ConstPtr& msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic_name);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Timer sync_timer_;
    ros::Publisher synced_pub_;

    // ROS Subscribers
    ros::Subscriber anchor_sub_;
    std::vector<ros::Subscriber> image_subs_;
    ros::Subscriber objects_sub_;
    ros::Subscriber uaq_sub_;

    // Parameters loaded from YAML
    double sync_rate_hz_;
    double time_slop_ms_;
    bool enable_virtual_heartbeat_;
    int max_heartbeat_cycles_;
    double watchdog_timeout_;
    std::string anchor_topic_;
    std::vector<std::string> input_topics_;

    // Core synchronization logic instances
    carmaker_gateway::MonotonicAnchorCache<carmaker_msgs::DynamicsInfo> anchor_cache_;
    std::map<std::string, std::shared_ptr<carmaker_gateway::LockFreeTimeRingBuffer<sensor_msgs::Image>>> image_caches_;
    carmaker_gateway::LockFreeTimeRingBuffer<carmaker_msgs::Objects> objects_cache_;
    carmaker_gateway::LockFreeTimeRingBuffer<carmaker_msgs::UAQ_Out> uaq_cache_;

    // Enterprise state tracking
    double last_valid_anchor_time_ = 0.0;
    carmaker_msgs::SyncedDataPtr out_msg_; // Persistent pre-allocated message
    int heartbeat_counter_ = 0;
};

} // namespace carmaker_gateway

#endif // CARMAKER_GATEWAY_NODELET_H
