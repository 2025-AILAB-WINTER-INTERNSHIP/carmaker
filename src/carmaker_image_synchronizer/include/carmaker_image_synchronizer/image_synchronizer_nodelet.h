#ifndef CARMAKER_IMAGE_SYNCHRONIZER_NODELET_H
#define CARMAKER_IMAGE_SYNCHRONIZER_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <mutex>
#include <vector>
#include <memory>

namespace carmaker_image_synchronizer {

/**
 * @brief Enterprise-grade Image Synchronizer for Multi-Camera perception.
 * Synchronizes multiple camera streams and attaches the latest CameraInfo.
 */
class ImageSynchronizerNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();

private:
    struct CameraChannel {
        std::string name;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub;
        ros::Subscriber info_sub;
        ros::Publisher img_pub;
        ros::Publisher info_pub;

        // Caching
        sensor_msgs::CameraInfo last_info;
        bool has_info = false;
        ros::Time last_info_time;
    };

    // Sync Callback
    void syncCallback(const sensor_msgs::ImageConstPtr& front,
                      const sensor_msgs::ImageConstPtr& rear,
                      const sensor_msgs::ImageConstPtr& left,
                      const sensor_msgs::ImageConstPtr& right);

    void publishWithSync(size_t index, const sensor_msgs::ImageConstPtr& img, const ros::Time& sync_time);

    // ROS Infrastructure
    ros::NodeHandle nh_, pnh_;
    
    // Channels
    std::vector<CameraChannel> channels_;
    std::mutex info_mutex_;

    // Settings
    size_t master_index_ = 0;
    double info_timeout_ = 2.0;

    // Message Filters
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image,
        sensor_msgs::Image, sensor_msgs::Image
    > SyncPolicy;
    
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

} // namespace carmaker_image_synchronizer

#endif
