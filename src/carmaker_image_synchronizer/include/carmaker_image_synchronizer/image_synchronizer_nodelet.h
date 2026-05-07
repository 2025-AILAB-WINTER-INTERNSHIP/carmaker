#ifndef CARMAKER_IMAGE_SYNCHRONIZER_NODELET_H
#define CARMAKER_IMAGE_SYNCHRONIZER_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <mutex>
#include <vector>
#include <memory>
#include <atomic>

namespace carmaker_image_synchronizer {

/**
 * @brief Image Synchronizer with Real-time Diagnostics.
 * 
 * Provides deterministic time-alignment for N-channel camera streams using 
 * master-clock restamping. Manages temporally aligned CameraInfo caching 
 * and provides health metrics via the ROS diagnostic system.
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

        // Diagnostics
        std::atomic<uint64_t> received_count{0};
        std::atomic<double> last_slop{0.0};
    };

    // Callbacks
    void syncCallback(const sensor_msgs::ImageConstPtr& front,
                      const sensor_msgs::ImageConstPtr& rear,
                      const sensor_msgs::ImageConstPtr& left,
                      const sensor_msgs::ImageConstPtr& right);

    void imageRawCallback(const sensor_msgs::ImageConstPtr& msg, size_t index);
    void publishWithSync(size_t index, const sensor_msgs::ImageConstPtr& img, const ros::Time& sync_time);
    void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

    // ROS Infrastructure
    ros::NodeHandle nh_, pnh_;
    
    // Channels
    std::vector<CameraChannel> channels_;
    std::mutex info_mutex_;

    // Advanced Settings
    size_t master_index_ = 0;
    double info_timeout_ = 2.0;

    // Diagnostics
    diagnostic_updater::Updater diagnostic_updater_;
    std::atomic<uint64_t> total_synced_count_{0};

    // Message Filters
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image,
        sensor_msgs::Image, sensor_msgs::Image
    > SyncPolicy;
    
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

} // namespace carmaker_image_synchronizer

#endif
