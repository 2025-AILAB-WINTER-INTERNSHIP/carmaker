#include "carmaker_localization/visualizer.h"
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

namespace carmaker_localization {

Visualizer::Visualizer(const ros::NodeHandle& nh) : nh_(nh) {
    ros::NodeHandle pnh("~");

    // 1. Frame configuration (Read these first)
    global_frame_ = pnh.param("frames/global", std::string("map"));
    prediction_frame_ = pnh.param("frames/prediction", std::string("Fr1A_pred"));
    gt_path_.header.frame_id = global_frame_;
    pred_path_.header.frame_id = global_frame_;

    // 2. Topic names from parameters
    std::string topic_svm = pnh.param("topics/publish/debug/svm_image", std::string("/localization/debug/svm_image"));
    std::string features_prefix = pnh.param("topics/publish/features_prefix", std::string("/localization/features"));
    std::string topic_est_markers = pnh.param("topics/publish/debug/estimated_markers", std::string("/localization/debug/estimated_markers"));
    std::string topic_obs_markers = pnh.param("topics/publish/debug/observed_markers", std::string("/localization/debug/observed_markers"));

    // 3. Publishers
    svm_pub_ = nh_.advertise<sensor_msgs::Image>(topic_svm, 1);
    feature_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(features_prefix + "/point_cloud", 1);
    est_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_est_markers, 1);
    obs_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(topic_obs_markers, 1);
    gt_path_pub_ = nh_.advertise<nav_msgs::Path>(topic_gt_path, 1);
    pred_path_pub_ = nh_.advertise<nav_msgs::Path>(topic_pred_path, 1);

    // 4. Vehicle dimensions
    front_edge_ = pnh.param("vehicle/front_edge", 4.68);
    rear_edge_ = pnh.param("vehicle/rear_edge", 0.0);
    width_ = pnh.param("vehicle/width", 1.8);
    length_ = pnh.param("vehicle/length", front_edge_ - rear_edge_);

    resolution_ = pnh.param("feature_extractor/bev/resolution", 0.05);
}

void Visualizer::publishSvmImage(const cv::Mat& svm_image) {
    if (svm_pub_.getNumSubscribers() > 0 && !svm_image.empty()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = prediction_frame_;
        cv_bridge::CvImage cv_img(header, "bgr8", svm_image);
        svm_pub_.publish(cv_img.toImageMsg());
    }
}

void Visualizer::publishFeatures(const carmaker_msgs::LocalFeatures& features) {
    if (feature_marker_pub_.getNumSubscribers() == 0 || features.features.empty()) return;

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker clear_marker;
    clear_marker.header = features.header;
    clear_marker.ns = "features_" + features.camera_name;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int id = 0;
    for (const auto& feat : features.features) {
        visualization_msgs::Marker m;
        m.header = features.header;
        m.ns = "features_" + features.camera_name;
        m.id = id++;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;

        m.pose.position.x = feat.x;
        m.pose.position.y = feat.y;
        m.pose.position.z = 0.0;
        m.pose.orientation.w = 1.0;

        m.scale.x = resolution_;
        m.scale.y = resolution_;
        m.scale.z = 0.01; // 바닥에 납작하게 붙은 형태로 시각화

        if (feat.class_id == 1) { // Lane
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8;
        } else if (feat.class_id == 2) { // Landmark
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8;
        } else {
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.5;
        }

        marker_array.markers.push_back(m);

        // Covariance Ellipse Marker
        visualization_msgs::Marker cov;
        cov.header = features.header;
        cov.ns = "features_cov_" + features.camera_name;
        cov.id = id++;
        cov.type = visualization_msgs::Marker::CYLINDER;
        cov.action = visualization_msgs::Marker::ADD;

        double c_xx = feat.covariance[0];
        double c_xy = feat.covariance[1];
        double c_yy = feat.covariance[3];

        double trace = c_xx + c_yy;
        double det = c_xx * c_yy - c_xy * c_xy;
        double det_term = std::max(0.0, std::pow(trace / 2.0, 2) - det);
        double l1 = trace / 2.0 + std::sqrt(det_term);
        double l2 = trace / 2.0 - std::sqrt(det_term);
        double angle = 0.5 * std::atan2(2.0 * c_xy, c_xx - c_yy);

        cov.pose.position.x = feat.x;
        cov.pose.position.y = feat.y;
        cov.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        cov.pose.orientation.x = q.x();
        cov.pose.orientation.y = q.y();
        cov.pose.orientation.z = q.z();
        cov.pose.orientation.w = q.w();

        cov.scale.x = std::sqrt(std::max(0.001, l1)) * 2.0;
        cov.scale.y = std::sqrt(std::max(0.001, l2)) * 2.0;
        cov.scale.z = 0.005;

        cov.color = m.color;
        cov.color.a = 0.15;

        marker_array.markers.push_back(cov);
    }

    feature_marker_pub_.publish(marker_array);
}

void Visualizer::publishEstimatedState(const geometry_msgs::PoseWithCovarianceStamped& pose) {
    if (est_marker_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker m;
    m.header = pose.header;
    m.ns = "ekf_pose";
    m.id = 0;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose = pose.pose.pose;
    m.scale.x = 2.0;
    m.scale.y = 0.5;
    m.scale.z = 0.5;
    m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
    marker_array.markers.push_back(m);

    visualization_msgs::Marker cov;
    cov.header = pose.header;
    cov.ns = "ekf_covariance";
    cov.id = 1;
    cov.type = visualization_msgs::Marker::CYLINDER;
    cov.action = visualization_msgs::Marker::ADD;
    cov.pose = pose.pose.pose;

    cov.color.r = 0.0; cov.color.g = 1.0; cov.color.b = 0.0; cov.color.a = 0.3;

    // Set scale based on actual covariance (3-sigma confidence interval)
    cov.scale.x = std::sqrt(std::max(0.01, pose.pose.covariance[0])) * 2.0; // P(x,x)
    cov.scale.y = std::sqrt(std::max(0.01, pose.pose.covariance[7])) * 2.0; // P(y,y)
    cov.scale.z = 0.05;

    marker_array.markers.push_back(cov);

    // 3. Vehicle Body Box (Current State)
    visualization_msgs::Marker body;
    body.header = pose.header;
    body.ns = "ekf_body";
    body.id = 2;
    body.type = visualization_msgs::Marker::CUBE;
    body.action = visualization_msgs::Marker::ADD;

    // Calculate center of the car body relative to Fr1A
    double center_offset = (front_edge_ + rear_edge_) / 2.0;
    double yaw = tf2::getYaw(pose.pose.pose.orientation);

    body.pose.position.x = pose.pose.pose.position.x + center_offset * std::cos(yaw);
    body.pose.position.y = pose.pose.pose.position.y + center_offset * std::sin(yaw);
    body.pose.position.z = 0.75; // Half of height
    body.pose.orientation = pose.pose.pose.orientation;

    body.scale.x = length_;
    body.scale.y = width_;
    body.scale.z = 1.5; // Standard car height
    body.color.r = 0.0; body.color.g = 1.0; body.color.b = 0.0; body.color.a = 0.4;
    marker_array.markers.push_back(body);

    est_marker_pub_.publish(marker_array);

    // 4. Update Predicted Trajectory (Distance-based filtering)
    double dx = pose.pose.pose.position.x - last_pred_pose_.position.x;
    double dy = pose.pose.pose.position.y - last_pred_pose_.position.y;
    if (pred_path_.poses.empty() || (dx*dx + dy*dy) > 0.2*0.2) {
        geometry_msgs::PoseStamped ps;
        ps.header = pose.header;
        ps.pose = pose.pose.pose;
        pred_path_.header.stamp = pose.header.stamp;
        pred_path_.poses.push_back(ps);
        last_pred_pose_ = ps.pose;

        if (pred_path_.poses.size() > 2000) pred_path_.poses.erase(pred_path_.poses.begin());
        pred_path_pub_.publish(pred_path_);
    }
}

void Visualizer::publishCorrectionState(const geometry_msgs::PoseWithCovarianceStamped& pose) {
    if (obs_marker_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::Marker m;
    m.header = pose.header;
    m.ns = "map_correction_ghost";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;

    // Center offset calculation for Ghost box
    double center_offset = (front_edge_ + rear_edge_) / 2.0;
    double yaw = tf2::getYaw(pose.pose.pose.orientation);

    m.pose.position.x = pose.pose.pose.position.x + center_offset * std::cos(yaw);
    m.pose.position.y = pose.pose.pose.position.y + center_offset * std::sin(yaw);
    m.pose.position.z = 0.75;
    m.pose.orientation = pose.pose.pose.orientation;

    m.scale.x = length_;
    m.scale.y = width_;
    m.scale.z = 1.5;
    m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.4; // Red for correction

    obs_marker_pub_.publish(m);
}

void Visualizer::publishGtPose(const carmaker_msgs::DynamicsInfo& gt) {
    // Distance-based filtering for GT Path
    double dx = gt.Car_x - last_gt_pose_.position.x;
    double dy = gt.Car_y - last_gt_pose_.position.y;

    if (gt_path_.poses.empty() || (dx*dx + dy*dy) > 0.2*0.2) {
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ros::Time::now();
        ps.header.frame_id = global_frame_;
        ps.pose.position.x = gt.Car_x;
        ps.pose.position.y = gt.Car_y;
        ps.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, gt.Car_Yaw);
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();

        gt_path_.header.stamp = ps.header.stamp;
        gt_path_.poses.push_back(ps);
        last_gt_pose_ = ps.pose;

        if (gt_path_.poses.size() > 2000) gt_path_.poses.erase(gt_path_.poses.begin());
        gt_path_pub_.publish(gt_path_);
    }
}

} // namespace carmaker_localization
