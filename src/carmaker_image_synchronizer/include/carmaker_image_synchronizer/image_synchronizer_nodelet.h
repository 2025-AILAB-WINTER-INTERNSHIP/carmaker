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

namespace carmaker_image_synchronizer {

class ImageSynchronizerNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();

private:
    // Sync Callback
    void syncCallback(const sensor_msgs::ImageConstPtr& front,
                      const sensor_msgs::ImageConstPtr& rear,
                      const sensor_msgs::ImageConstPtr& left,
                      const sensor_msgs::ImageConstPtr& right);

    // ROS Infrastructure
    ros::NodeHandle nh_, pnh_;
    
    // 8-Channel Synced Publishers
    ros::Publisher pub_front_img_, pub_rear_img_, pub_left_img_, pub_right_img_;
    ros::Publisher pub_front_info_, pub_rear_info_, pub_left_info_, pub_right_info_;

    // Message Filters
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image,
        sensor_msgs::Image, sensor_msgs::Image
    > SyncPolicy;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_front_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_rear_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_left_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_right_;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // Camera Info Caching
    std::mutex info_mutex_;
    bool has_info_front_ = false, has_info_rear_ = false, has_info_left_ = false, has_info_right_ = false;

    ros::Subscriber info_front_sub_, info_rear_sub_, info_left_sub_, info_right_sub_;
    sensor_msgs::CameraInfo last_info_front_, last_info_rear_, last_info_left_, last_info_right_;
};

} // namespace carmaker_image_synchronizer

#endif
