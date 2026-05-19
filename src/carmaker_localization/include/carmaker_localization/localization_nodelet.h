#ifndef CARMAKER_LOCALIZATION_NODELET_H
#define CARMAKER_LOCALIZATION_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <mutex>
#include <atomic>
#include <memory>
#include <cmath>
#include <array>
#include <map>
#include <vector>
#include <cv_bridge/cv_bridge.h>

#include "carmaker_localization/feature_extractor.h"
#include "carmaker_localization/ekf_core.h"
#include "carmaker_localization/visualizer.h"
#include "carmaker_localization/map_loader_base.h"
#include "carmaker_localization/matcher_base.h"
#include <carmaker_msgs/LocalFeatures.h>
#include <carmaker_msgs/DynamicsInfo.h>
#include <carmaker_msgs/CameraBundle.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace carmaker_localization {

struct Channel {
    std::string name;
    std::string frame;
    std::string input_topic;
    std::string info_topic;
    std::shared_ptr<FeatureExtractor> extractor;
    std::atomic<bool> lut_initialized{false};
    std::shared_ptr<std::mutex> lut_mutex = std::make_shared<std::mutex>();
    std::vector<double> bev_x_range;
    std::vector<double> bev_y_range;

    std::atomic<uint64_t> processed_count{0};

    Channel() = default;
    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;
    Channel(Channel&& other) noexcept :
        name(std::move(other.name)),
        frame(std::move(other.frame)),
        input_topic(std::move(other.input_topic)),
        info_topic(std::move(other.info_topic)),
        extractor(std::move(other.extractor)),
        lut_initialized(other.lut_initialized.load()),
        lut_mutex(std::move(other.lut_mutex)),
        bev_x_range(std::move(other.bev_x_range)),
        bev_y_range(std::move(other.bev_y_range)),
        processed_count(other.processed_count.load()) {}
};

class LocalizationNodelet : public nodelet::Nodelet {
public:
    LocalizationNodelet() = default;
    virtual ~LocalizationNodelet() = default;

private:
    // Lifecycle functions (SRP breakdown of giant onInit)
    virtual void onInit() override;
    bool loadParameters();
    void initEkf();
    void initSvm();
    void setupRosIo();

    // Callbacks
    void dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg);
    void predictionCallback(const ros::TimerEvent& event);
    void publishEstimation(const ros::Time& stamp);
    void imagesCallback(
        const sensor_msgs::ImageConstPtr& img0,
        const sensor_msgs::ImageConstPtr& img1,
        const sensor_msgs::ImageConstPtr& img2,
        const sensor_msgs::ImageConstPtr& img3);
    void bundleCallback(const carmaker_msgs::CameraBundleConstPtr& msg);
    void processImages(
        const std::array<sensor_msgs::ImageConstPtr, 4>& imgs,
        const std::array<sensor_msgs::CameraInfoConstPtr, 4>& infos);

    void performCorrection(const carmaker_msgs::LocalFeatures& features);
    void infoCallback(const sensor_msgs::CameraInfoConstPtr& msg, size_t idx);
    void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

    // =========================================================================
    // 1. Hardware Constants
    // =========================================================================
    static constexpr size_t NUM_CAMERAS = 4;

    // =========================================================================
    // 2. Read-Only Configuration Parameters
    // =========================================================================
    int imu_id_ = 0;
    std::string global_frame_;
    std::string prediction_frame_;
    double search_radius_ = 20.0;
    double tire_radius_ = 0.327;
    double fitness_threshold_ = 0.5;
    bool map_matcher_enabled_ = true;  // map_matcher/enable 파라미터로 제어

    // SVM Config
    double svm_res_ = 0.05;
    double svm_x_max_ = 7.68;
    double svm_x_min_ = -3.0;
    double svm_y_max_ = 3.0;
    double svm_y_min_ = -3.0;

    // Feature Extractor Config
    std::string image_type_;
    double r_max_ = 15.0;
    double cov_k_ = 1.0;

    // Map Loader Config
    double resolution_ = 0.05;

    // Vehicle Kinematics
    double tire_radius_ = 0.327;
    double track_width_ = 1.655;
    double rear_axle_x_ = 0.79;

    // EKF Noise Config
    double wheel_speed_std_ = 0.05;
    double imu_acc_std_ = 0.1;
    double imu_gyro_std_ = 0.01;

    // EKF Initial State Config
    bool use_manual_initial_state_ = false;
    double init_x_ = 0.0;
    double init_y_ = 0.0;
    double init_yaw_ = 0.0;

    // =========================================================================
    // 3. ROS Communication Objects
    // =========================================================================
    ros::Subscriber dynamics_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher estimation_data_pub_;
    ros::Publisher correction_data_pub_;
    diagnostic_updater::Updater diagnostic_updater_;
    std::map<std::string, ros::Publisher> feature_data_pubs_;
    ros::Timer prediction_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<Visualizer> visualizer_;

    // Camera Subscriptions
    bool use_bundle_ = false;
    ros::Subscriber bundle_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> seg_subs_[NUM_CAMERAS];
    ros::Subscriber info_subs_[NUM_CAMERAS];

    typedef message_filters::sync_policies::ExactTime<
        sensor_msgs::Image, sensor_msgs::Image,
        sensor_msgs::Image, sensor_msgs::Image> SyncPolicy4;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy4>> sync_all_;

    // =========================================================================
    // 4. Algorithm Core Engines
    // =========================================================================
    std::shared_ptr<EkfCore> ekf_core_;
    std::shared_ptr<MapLoaderBase> map_loader_;
    std::shared_ptr<MatcherBase> matcher_;

    // =========================================================================
    // 5. Runtime Shared States & Sync Buffers
    // =========================================================================
    std::vector<Channel> channels_;
    bool fusion_ = false;
    double last_prediction_time_ = 0.0;

    // EKF & Dynamics synchronization
    std::mutex estimation_mutex_;
    std::mutex dyn_mutex_;
    carmaker_msgs::DynamicsInfo latest_dynamics_;
    bool dynamics_received_ = false;

    // CameraInfo buffer protected by mutex to prevent Data Race
    std::mutex info_array_mutex_;
    sensor_msgs::CameraInfoConstPtr latest_infos_[NUM_CAMERAS];

    // SVM Canvas & Masks
    cv::Mat svm_canvas_;
    std::mutex svm_mutex_;
    std::map<std::string, cv::Mat> svm_masks_;
    std::vector<cv::Point> seam_line_points_;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_NODELET_H
