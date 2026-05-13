#include "carmaker_localization/feature_extractor.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <sensor_msgs/image_encodings.h>

namespace carmaker_localization {

FeatureExtractor::FeatureExtractor(const ros::NodeHandle& nh, const std::string& camera_name,
                                    const std::vector<double>& x_range, const std::vector<double>& y_range)
    : nh_(nh), lut_initialized_(false), has_optimal_point_(false), camera_name_(camera_name) {
    ros::NodeHandle pnh("~");

    if (x_range.size() >= 2) {
        bev_cfg_.x_min = x_range[0];
        bev_cfg_.x_max = x_range[1];
    } else {
        bev_cfg_.x_min = -10.0; bev_cfg_.x_max = 10.0;
    }

    if (y_range.size() >= 2) {
        bev_cfg_.y_min = y_range[0];
        bev_cfg_.y_max = y_range[1];
    } else {
        bev_cfg_.y_min = -5.0; bev_cfg_.y_max = 15.0;
    }

    bev_cfg_.resolution = pnh.param("feature_extractor/bev/resolution", 0.05);

    bev_cfg_.width = std::ceil((bev_cfg_.x_max - bev_cfg_.x_min) / bev_cfg_.resolution);
    bev_cfg_.height = std::ceil((bev_cfg_.y_max - bev_cfg_.y_min) / bev_cfg_.resolution);
}

void FeatureExtractor::updateLUT(const sensor_msgs::CameraInfoConstPtr& info,
                                    const geometry_msgs::TransformStamped& tf) {
    std::vector<cv::Point3f> object_points;
    cartesian_lut_x_ = cv::Mat(bev_cfg_.height, bev_cfg_.width, CV_32F);
    cartesian_lut_y_ = cv::Mat(bev_cfg_.height, bev_cfg_.width, CV_32F);

    for (int v = 0; v < bev_cfg_.height; ++v) {
        for (int u = 0; u < bev_cfg_.width; ++u) {
            double x = bev_cfg_.x_min + (u + 0.5) * bev_cfg_.resolution;
            double y = bev_cfg_.y_max - (v + 0.5) * bev_cfg_.resolution;
            object_points.emplace_back(x, y, 0.0);
            cartesian_lut_x_.at<float>(v, u) = static_cast<float>(x);
            cartesian_lut_y_.at<float>(v, u) = static_cast<float>(y);
        }
    }

    std::vector<cv::Point3f> cam_points;
    cam_points.reserve(object_points.size());
    for (const auto& pt : object_points) {
        geometry_msgs::Point pt_base;
        pt_base.x = pt.x; pt_base.y = pt.y; pt_base.z = pt.z;
        geometry_msgs::Point pt_cam;
        tf2::doTransform(pt_base, pt_cam, tf);
        cam_points.emplace_back(pt_cam.x, pt_cam.y, pt_cam.z);
    }

    cv::Mat K(3, 3, CV_64F, 0.0);
    for (int i = 0; i < 9; ++i) K.at<double>(i / 3, i % 3) = info->K[i];
    cv::Mat D;
    if (!info->D.empty()) {
        D = cv::Mat(info->D.size(), 1, CV_64F);
        for (size_t i = 0; i < info->D.size(); ++i) D.at<double>(i, 0) = info->D[i];
    } else {
        D = cv::Mat::zeros(4, 1, CV_64F);
    }

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    std::vector<cv::Point2f> image_points;

    if (info->distortion_model == "equidistant" || info->distortion_model == "fisheye") {
        cv::fisheye::projectPoints(cam_points, image_points, rvec, tvec, K, D);
    } else {
        cv::projectPoints(cam_points, rvec, tvec, K, D, image_points);
    }

    map1_ = cv::Mat(bev_cfg_.height, bev_cfg_.width, CV_32FC1);
    map2_ = cv::Mat(bev_cfg_.height, bev_cfg_.width, CV_32FC1);
    for (int v = 0; v < bev_cfg_.height; ++v) {
        for (int u = 0; u < bev_cfg_.width; ++u) {
            int idx = v * bev_cfg_.width + u;
            map1_.at<float>(v, u) = image_points[idx].x;
            map2_.at<float>(v, u) = image_points[idx].y;
        }
    }
    // Calculate Optimal Ground Point (Sweet Spot)
    // Ray from camera center (0,0,0) along Z-axis (0,0,1) in camera optical frame
    tf2::Transform tf_base2cam;
    tf2::fromMsg(tf.transform, tf_base2cam);
    tf2::Transform tf_cam2base = tf_base2cam.inverse();

    tf2::Vector3 origin_in_base = tf_cam2base * tf2::Vector3(0,0,0);
    tf2::Vector3 ray_dir_in_base = tf_cam2base.getBasis() * tf2::Vector3(0,0,1);

    // Intersection with Z=0 plane: origin.z + lambda * ray_dir.z = 0
    if (std::abs(ray_dir_in_base.z()) > 1e-3) {
        double lambda = -origin_in_base.z() / ray_dir_in_base.z();
        if (lambda > 0) {
            optimal_point_.x = origin_in_base.x() + lambda * ray_dir_in_base.x();
            optimal_point_.y = origin_in_base.y() + lambda * ray_dir_in_base.y();
            has_optimal_point_ = true;
            NODELET_INFO("[%s] Optimal ground focus point calculated at: (%.2f, %.2f)",
                            camera_name_.c_str(), optimal_point_.x, optimal_point_.y);
        } else {
            has_optimal_point_ = false;
        }
    } else {
        has_optimal_point_ = false;
    }

    lut_initialized_ = true;
}

carmaker_msgs::LocalFeatures FeatureExtractor::process(
    const sensor_msgs::ImageConstPtr& seg_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg,
    cv::Mat& out_bev_image) {

    carmaker_msgs::LocalFeatures features;
    features.header = seg_msg->header;
    features.camera_name = camera_name_;

    if (!lut_initialized_) {
        ROS_WARN_THROTTLE(1.0, "LUT not initialized yet.");
        return features;
    }

    cv_bridge::CvImagePtr cv_seg;
    try {
        cv_seg = cv_bridge::toCvCopy(seg_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return features;
    }

    cv::Mat bev_class;
    cv::remap(cv_seg->image, bev_class, map1_, map2_, cv::INTER_NEAREST);

    out_bev_image = cv::Mat::zeros(bev_cfg_.height, bev_cfg_.width, CV_8UC3);

    cv::Mat mask = bev_class > 0;
    if (cv::countNonZero(mask) == 0) return features;

    ros::NodeHandle pnh("~");
    double r_max = pnh.param("feature_extractor/extraction/r_max", 15.0);
    double cov_k = pnh.param("feature_extractor/extraction/covariance_k", 1.0);

    std::vector<cv::Point> non_zero_points;
    cv::findNonZero(mask, non_zero_points);
    features.features.reserve(non_zero_points.size());

    for (const auto& pt : non_zero_points) {
        int u = pt.x;
        int v = pt.y;
        uint8_t class_id = bev_class.at<uint8_t>(v, u);

        if (class_id == 1) out_bev_image.at<cv::Vec3b>(v, u) = cv::Vec3b(255, 255, 255); // White
        else if (class_id == 2) out_bev_image.at<cv::Vec3b>(v, u) = cv::Vec3b(76, 76, 255); // Coral

        float x = cartesian_lut_x_.at<float>(v, u);
        float y = cartesian_lut_y_.at<float>(v, u);
        float r = std::sqrt(x*x + y*y);

        if (r > r_max) continue;
        if (r < 0.01) continue;

        float e_r_x = x / r;
        float e_r_y = y / r;
        float e_t_x = -y / r;
        float e_t_y = x / r;

        float dx_opt = (has_optimal_point_) ? (x - optimal_point_.x) : 0.0f;
        float dy_opt = (has_optimal_point_) ? (y - optimal_point_.y) : 0.0f;
        float dist_opt_sq = dx_opt*dx_opt + dy_opt*dy_opt;

        float base_sigma_m_per_px = bev_cfg_.resolution;
        float sigma_r = base_sigma_m_per_px * (1.0f + cov_k * dist_opt_sq);
        float sigma_r_sq = sigma_r * sigma_r;
        float sigma_t_sq = bev_cfg_.resolution * bev_cfg_.resolution;

        carmaker_msgs::LocalFeature f_msg;
        f_msg.x = x;
        f_msg.y = y;
        f_msg.cov_xx = sigma_r_sq * e_r_x * e_r_x + sigma_t_sq * e_t_x * e_t_x;
        f_msg.cov_xy = sigma_r_sq * e_r_x * e_r_y + sigma_t_sq * e_t_x * e_t_y;
        f_msg.cov_yy = sigma_r_sq * e_r_y * e_r_y + sigma_t_sq * e_t_y * e_t_y;
        f_msg.class_id = class_id;

        features.features.push_back(f_msg);
    }

    return features;
}

} // namespace carmaker_localization
