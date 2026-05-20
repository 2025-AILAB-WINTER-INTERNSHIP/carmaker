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
#include <carmaker_msgs/CameraBundle.h>

#include <mutex>
#include <array>
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
    // Channel indexes
    static constexpr size_t FRONT = 0;
    static constexpr size_t REAR = 1;
    static constexpr size_t LEFT = 2;
    static constexpr size_t RIGHT = 3;
    static constexpr size_t NUM_CHANNELS = 4;

    struct CameraChannel {
        std::string name;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub;
        ros::Subscriber info_sub;
        ros::Publisher img_pub;
        ros::Publisher info_pub;

        // Caching (Protected by info_mutex_)
        sensor_msgs::CameraInfo last_info;
        bool has_info = false;
        ros::Time last_info_time;

        // Diagnostics
        // Using atomic for high-frequency counter to avoid lock contention in imageRawCallback
        std::atomic<uint64_t> received_count{0};

        // C++20 optimization note: std::atomic<double> could be used here.
        // In C++17, we protect this with status_mutex_.
        double last_slop = 0.0;

        // CameraChannel needs custom move constructor because std::atomic is non-copyable/non-movable
        CameraChannel() = default;
        CameraChannel(const CameraChannel&) = delete;
        CameraChannel& operator=(const CameraChannel&) = delete;
    };

    // Callbacks
    void imageRawCallback(const sensor_msgs::ImageConstPtr& msg, size_t index);
    void syncCallback(const sensor_msgs::ImageConstPtr& front,
                        const sensor_msgs::ImageConstPtr& rear,
                        const sensor_msgs::ImageConstPtr& left,
                        const sensor_msgs::ImageConstPtr& right);
    void timerCallback(const ros::TimerEvent& event);

    // Helpers
    void resetSynchronizer();
    void checkTimeJump(const ros::Time& current_time);
    bool getValidCameraInfo(size_t index, const ros::Time& sync_time, sensor_msgs::CameraInfo& out_info);
    void processSyncedImages(const std::array<sensor_msgs::ImageConstPtr, 4>& images);
    void publishWithSync(size_t index, const sensor_msgs::ImageConstPtr& img, const ros::Time& sync_time);
    void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

    // ROS Infrastructure
    ros::NodeHandle nh_, pnh_;
    ros::Timer diag_timer_;
    ros::Publisher bundle_pub_;

    // Channels, Data
    std::array<CameraChannel, NUM_CHANNELS> channels_;

    // Mutex splitting to reduce contention:
    // 1. info_mutex_: Protects CameraInfo caching and its validity state.
    // 2. status_mutex_: Protects diagnostic status fields (slop) and master settings.
    std::mutex info_mutex_;
    std::mutex status_mutex_;

    // Advanced Settings
    bool use_bundle_ = false;
    size_t master_index_ = 0;
    int queue_size_ = 10;
    double slop_ = 0.05;
    double info_timeout_ = 2.0;

    // Time Jump Tracking
    ros::Time last_image_time_;
    std::mutex time_mutex_;

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
