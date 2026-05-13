#include "carmaker_localization/visualizer.h"
#include <cv_bridge/cv_bridge.h>

namespace carmaker_localization {

Visualizer::Visualizer(const ros::NodeHandle& nh) : nh_(nh) {
    ros::NodeHandle pnh("~");
    std::string topic_svm = pnh.param("topics/publish/svm_image", std::string("/localization/debug/svm_image"));
    std::string features_prefix = pnh.param("topics/publish/features_prefix", std::string("/localization/features"));

    svm_pub_ = nh_.advertise<sensor_msgs::Image>(topic_svm, 1);
    feature_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(features_prefix + "/points_overlay", 1);
    ekf_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/localization/debug/ekf_markers", 1);
    map_match_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/localization/debug/map_match_overlay", 1);
}

void Visualizer::publishSvmImage(const cv::Mat& svm_image) {
    if (svm_pub_.getNumSubscribers() > 0 && !svm_image.empty()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "base_link";
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
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;

        m.pose.position.x = feat.x;
        m.pose.position.y = feat.y;
        m.pose.position.z = 0.0;
        m.pose.orientation.w = 1.0;

        m.scale.x = 0.2;
        m.scale.y = 0.2;
        m.scale.z = 0.2;

        if (feat.class_id == 1) { // Lane
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8;
        } else if (feat.class_id == 2) { // Landmark
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8;
        } else {
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.5;
        }

        marker_array.markers.push_back(m);
    }

    feature_marker_pub_.publish(marker_array);
}

void Visualizer::publishEkfState(const geometry_msgs::PoseWithCovarianceStamped& pose) {
    if (ekf_marker_pub_.getNumSubscribers() == 0) return;

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

    cov.scale.x = std::sqrt(pose.pose.covariance[0]) * 2.0;
    cov.scale.y = std::sqrt(pose.pose.covariance[7]) * 2.0;
    cov.scale.z = 0.01;
    cov.color.r = 0.0; cov.color.g = 1.0; cov.color.b = 0.0; cov.color.a = 0.3;
    marker_array.markers.push_back(cov);

    ekf_marker_pub_.publish(marker_array);
}

void Visualizer::publishMapCorrection(const geometry_msgs::PoseWithCovarianceStamped& pose) {
    if (map_match_marker_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::Marker m;
    m.header = pose.header;
    m.ns = "map_correction_ghost";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose = pose.pose.pose;
    m.scale.x = 4.5;
    m.scale.y = 1.8;
    m.scale.z = 1.5;
    m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.4;

    map_match_marker_pub_.publish(m);
}

} // namespace carmaker_localization
