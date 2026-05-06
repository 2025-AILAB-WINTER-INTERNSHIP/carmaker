#ifndef CARMAKER_GATEWAY_NODELET_H
#define CARMAKER_GATEWAY_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <carmaker_msgs/DynamicsInfo.h>
#include <carmaker_msgs/Objects.h>
#include <carmaker_msgs/UAQ_Out.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <carmaker_msgs/SyncedData.h>
#include <carmaker_gateway/core/lock_free_time_ring_buffer.hpp>
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
    void cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic_name);
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, const std::string& topic_name);

    struct CameraContext {
        std::string name;
        std::shared_ptr<carmaker_gateway::LockFreeTimeRingBuffer<sensor_msgs::Image, 10>> image_cache;
        std::shared_ptr<carmaker_gateway::LockFreeTimeRingBuffer<sensor_msgs::CameraInfo, 10>> info_cache;
        ros::Subscriber image_sub;
        ros::Subscriber info_sub;
        std::atomic<double> last_published_stamp;

        CameraContext() : 
            image_cache(std::make_shared<carmaker_gateway::LockFreeTimeRingBuffer<sensor_msgs::Image, 10>>()),
            info_cache(std::make_shared<carmaker_gateway::LockFreeTimeRingBuffer<sensor_msgs::CameraInfo, 10>>()),
            last_published_stamp(0.0) {}
    };

    /**
     * @brief Core synchronization and publication logic.
     * @param target_time The timestamp to sync towards.
     * @param anchor The anchor data if available (optional).
     */
    void processSynchronization(double target_time, const std::shared_ptr<const carmaker_msgs::DynamicsInfo>& anchor);
    
    /**
     * @brief Perform a shallow copy of Image metadata, excluding the heavy data field.
     */
    void shallowCopyImageMetadata(const sensor_msgs::Image& src, sensor_msgs::Image& dst);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Timer sync_timer_;
    ros::Publisher synced_pub_;

    // ROS Subscribers
    ros::Subscriber anchor_sub_;
    ros::Subscriber objects_sub_;
    ros::Subscriber uaq_sub_;

    // Parameters loaded from YAML
    double sync_rate_hz_;
    double time_slop_ms_;
    bool enable_virtual_heartbeat_;
    bool use_anchor_trigger_;
    int max_heartbeat_cycles_;
    double watchdog_timeout_;
    double watchdog_ratio_;
    std::string qos_reliability_;
    std::string anchor_topic_;
    std::vector<std::string> input_topics_;
    std::string output_topic_;

    // Core synchronization logic instances
    carmaker_gateway::MonotonicAnchorCache<carmaker_msgs::DynamicsInfo, carmaker_gateway::TimestampExtractor<carmaker_msgs::DynamicsInfo>> anchor_cache_;
    carmaker_gateway::LockFreeTimeRingBuffer<carmaker_msgs::Objects, 20, carmaker_gateway::TimestampExtractor<carmaker_msgs::Objects>> objects_cache_;
    carmaker_gateway::LockFreeTimeRingBuffer<carmaker_msgs::UAQ_Out, 20, carmaker_gateway::TimestampExtractor<carmaker_msgs::UAQ_Out>> uaq_cache_;
    
    // Grouped Camera Management
    std::map<std::string, std::unique_ptr<CameraContext>> camera_contexts_;
    std::vector<CameraContext*> fast_camera_list_;

    // Enterprise state tracking
    double last_valid_anchor_time_ = 0.0;
    double last_anchor_arrival_time_ = 0.0;
    double time_slop_sec_ = 0.0;
    double base_virtual_time_ = 0.0;
    int heartbeat_counter_ = 0;
};

} // namespace carmaker_gateway

#endif // CARMAKER_GATEWAY_NODELET_H
