#ifndef CARMAKER_LOCALIZATION_FEATURE_EXTRACTOR_H
#define CARMAKER_LOCALIZATION_FEATURE_EXTRACTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <carmaker_msgs/LocalFeatures.h>

namespace carmaker_localization {

struct BevConfig {
    double x_min, x_max;
    double y_min, y_max;
    double resolution;
    int width, height;
};

class FeatureExtractor {
public:
    explicit FeatureExtractor(const ros::NodeHandle& nh, const std::string& camera_name,
                              const std::vector<double>& x_range, const std::vector<double>& y_range);
    ~FeatureExtractor() = default;

    /**
     * @brief Process incoming image and camera info
     */
    carmaker_msgs::LocalFeatures process(
        const sensor_msgs::ImageConstPtr& seg_msg,
        const sensor_msgs::CameraInfoConstPtr& info_msg,
        cv::Mat& out_bev_image);

private:
    void updateLUT(const sensor_msgs::CameraInfoConstPtr& info,
                    const geometry_msgs::TransformStamped& tf);

    ros::NodeHandle nh_;
    BevConfig bev_cfg_;

    // Look-Up Table for remap
    cv::Mat map1_, map2_;
    cv::Mat cartesian_lut_x_, cartesian_lut_y_; // Precomputed X,Y in base_link

    bool lut_initialized_;
    bool has_optimal_point_;
    cv::Point2f optimal_point_;
    std::string camera_name_;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_FEATURE_EXTRACTOR_H
