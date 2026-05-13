#ifndef CARMAKER_LOCALIZATION_NODELET_H
#define CARMAKER_LOCALIZATION_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <atomic>
#include <memory>
#include <cmath>

#include "carmaker_localization/feature_extractor.h"
#include "carmaker_localization/ekf_core.h"
#include "carmaker_localization/visualizer.h"
#include "carmaker_localization/map_loader_base.h"
#include "carmaker_localization/matcher_base.h"
#include <carmaker_msgs/LocalFeatures.h>
#include <carmaker_msgs/DynamicsInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace carmaker_localization {

struct Channel {
    std::string name;
    std::string frame;
    std::shared_ptr<FeatureExtractor> extractor;
    bool lut_initialized = false;
    std::vector<double> bev_x_range;
    std::vector<double> bev_y_range;

    ros::Publisher feature_pub;

    std::atomic<uint64_t> processed_count{0};

    Channel() = default;
    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;
    Channel(Channel&& other) noexcept :
        name(std::move(other.name)),
        frame(std::move(other.frame)),
        extractor(std::move(other.extractor)),
        lut_initialized(other.lut_initialized),
        bev_x_range(std::move(other.bev_x_range)),
        bev_y_range(std::move(other.bev_y_range)),
        feature_pub(std::move(other.feature_pub)),
        processed_count(other.processed_count.load()) {}
};

class LocalizationNodelet : public nodelet::Nodelet {
public:
    LocalizationNodelet() = default;
    virtual ~LocalizationNodelet() = default;

private:
    virtual void onInit() override;

    void dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg);
    void ekfTimerCallback(const ros::TimerEvent& event);
    void publishEkfState(const ros::Time& stamp);
    void imagesCallback(
        const sensor_msgs::ImageConstPtr& img0,
        const sensor_msgs::ImageConstPtr& img1,
        const sensor_msgs::ImageConstPtr& img2,
        const sensor_msgs::ImageConstPtr& img3);

    void performCorrection(const carmaker_msgs::LocalFeatures& features);
    void infoCallback(const sensor_msgs::CameraInfoConstPtr& msg, size_t idx);

    std::vector<Channel> channels_;
    bool fusion_ = false;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> seg_subs_[4];
    ros::Subscriber info_subs_[4];
    sensor_msgs::CameraInfoConstPtr latest_infos_[4];

    typedef message_filters::sync_policies::ExactTime<
        sensor_msgs::Image, sensor_msgs::Image,
        sensor_msgs::Image, sensor_msgs::Image> SyncPolicy4;

    std::unique_ptr<message_filters::Synchronizer<SyncPolicy4>> sync_all_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<Visualizer> visualizer_;

    std::shared_ptr<EkfCore> ekf_core_;
    ros::Subscriber dynamics_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    ros::Timer ekf_timer_;
    double last_ekf_time_ = 0.0;
    std::mutex ekf_mutex_;
    std::mutex dyn_mutex_;
    carmaker_msgs::DynamicsInfo latest_dynamics_;
    bool dynamics_received_ = false;

    std::shared_ptr<MapLoaderBase> map_loader_;
    std::shared_ptr<MatcherBase> matcher_;
    ros::Publisher map_correction_pub_;
    double search_radius_ = 20.0;

    cv::Mat svm_canvas_;
    std::mutex svm_mutex_;
    std::map<std::string, cv::Mat> svm_masks_; // Precomputed masks per camera
    double svm_res_ = 0.05;
    double svm_x_max_ = 7.68;
    double svm_x_min_ = -3.0;
    double svm_y_max_ = 3.0;
    double svm_y_min_ = -3.0;

    bool use_manual_initial_state_ = false;
    double init_x_ = 0.0, init_y_ = 0.0, init_yaw_ = 0.0;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_NODELET_H
