#include "carmaker_localization/visualizer.h"
#include <tf2/utils.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointField.h>

namespace carmaker_localization {

Visualizer::Visualizer(const ros::NodeHandle& nh) : nh_(nh) {
    ros::NodeHandle pnh("~");

    // 1. Frame configuration
    global_frame_ = pnh.param("frames/global", std::string("map"));
    prediction_frame_ = pnh.param("frames/prediction", std::string("Fr1A_pred"));
    estimation_trajectory_.header.frame_id = global_frame_;

    // 2. Topic names
    std::string topic_svm = pnh.param("topics/publish/debug/svm_image", std::string("/localization/debug/svm_image"));
    std::string topic_estimation_markers = pnh.param("topics/publish/debug/estimation_markers", std::string("/localization/debug/estimation_markers"));
    std::string topic_correction_markers = pnh.param("topics/publish/debug/correction_markers", std::string("/localization/debug/correction_markers"));
    std::string topic_estimation_trajectory = pnh.param("topics/publish/debug/estimation_trajectory", std::string("/localization/debug/estimation_trajectory"));
    std::string topic_map_features = pnh.param("topics/publish/debug/map_features", std::string("/localization/debug/map_features"));

    // 3. Publishers
    svm_pub_ = nh_.advertise<sensor_msgs::Image>(topic_svm, 1);
    estimation_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_estimation_markers, 1);
    correction_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_correction_markers, 1);
    estimation_trajectory_pub_ = nh_.advertise<nav_msgs::Path>(topic_estimation_trajectory, 1);
    map_features_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_map_features, 1, true); // latched = true for static map

    // 4. Vehicle dimensions
    vehicle_width_ = pnh.param("vehicle/width", 1.9);
    vehicle_length_ = pnh.param("vehicle/length", 4.635);
    vehicle_height_ = pnh.param("vehicle/height", 1.605);
    vehicle_length_offset_ = pnh.param("vehicle/center_offset_x", 2.3175);

    resolution_ = pnh.param("feature_extractor/bev/resolution", 0.05);
}

void Visualizer::publishSvmImage(const cv::Mat& svm_image) {
    if (svm_pub_.getNumSubscribers() > 0 && !svm_image.empty()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = prediction_frame_;
        cv_bridge::CvImage cv_img(header, "rgb8", svm_image);
        svm_pub_.publish(cv_img.toImageMsg());
    }
}

void Visualizer::publishObservation(const std::string& channel_name, const carmaker_msgs::LocalFeatures& features) {
    if (observation_pub_map_.find(channel_name) == observation_pub_map_.end()) {
        ros::NodeHandle pnh("~");
        std::string prefix = pnh.param("topics/publish/debug/observation_prefix", std::string("/localization/debug/observation"));
        std::string topic = prefix + "/" + channel_name;
        observation_pub_map_[channel_name] = nh_.advertise<visualization_msgs::MarkerArray>(topic, 1);
    }

    auto& pub = observation_pub_map_[channel_name];
    if (pub.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray marker_array;

    // Clear previous markers by publishing a DELETEALL marker for each namespace
    visualization_msgs::Marker clear_points;
    clear_points.header = features.header;
    clear_points.ns = "observation_points";
    clear_points.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_points);

    visualization_msgs::Marker clear_cov;
    clear_cov.header = features.header;
    clear_cov.ns = "observation_covariance";
    clear_cov.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_cov);

    int id = 0;

    for (const auto& feat : features.features) {
        // 1. Point Marker (Sphere)
        visualization_msgs::Marker m;
        m.header = features.header;
        m.ns = "observation_points";
        m.id = id++;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;

        m.pose.position.x = feat.x;
        m.pose.position.y = feat.y;
        m.pose.position.z = 0.01;
        m.pose.orientation.w = 1.0;

        m.scale.x = resolution_;
        m.scale.y = resolution_;
        m.scale.z = 0.01;

        if (feat.class_id == 1) { // Lane
            // Yellow
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8;
        } else if (feat.class_id == 2) { // Landmark
            // Cyan
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.8;
        } else {
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.5;
        }
        marker_array.markers.push_back(m);

        // 2. Covariance Marker (Cylinder)
        visualization_msgs::Marker cov;
        cov.header = features.header;
        cov.ns = "observation_covariance";
        cov.id = id++;
        cov.type = visualization_msgs::Marker::CYLINDER;
        cov.action = visualization_msgs::Marker::ADD;

        double cov_xx = feat.cov_xx;
        double cov_yy = feat.cov_yy;
        double cov_xy = feat.cov_xy;

        // 고유값(Eigenvalue) 분석을 통한 참값 타원 장축/단축 및 각도 계산
        double sum = cov_xx + cov_yy;
        double diff = cov_xx - cov_yy;
        double term = std::sqrt(diff * diff + 4.0 * cov_xy * cov_xy);
        double lambda1 = 0.5 * (sum + term);
        double lambda2 = 0.5 * (sum - term);

        double sigma_major = std::sqrt(std::max(0.001, lambda1));
        double sigma_minor = std::sqrt(std::max(0.001, lambda2));
        double angle = 0.5 * std::atan2(2.0 * cov_xy, diff);

        cov.pose.position.x = feat.x;
        cov.pose.position.y = feat.y;
        cov.pose.position.z = 0.005;

        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        cov.pose.orientation = tf2::toMsg(q);

        cov.scale.x = sigma_major * 2.0;
        cov.scale.y = sigma_minor * 2.0;
        cov.scale.z = 0.0025;

        cov.color = m.color;
        cov.color.a = 0.3;
        marker_array.markers.push_back(cov);
    }

    pub.publish(marker_array);
}

void Visualizer::publishEstimation(const geometry_msgs::PoseWithCovarianceStamped& pose) {
    if (estimation_marker_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray marker_array;

    // 1. Pose Marker
    visualization_msgs::Marker m;
    m.header = pose.header;
    m.ns = "estimation_pose";
    m.id = 0;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 1.0; m.scale.y = 0.1; m.scale.z = 0.1;
    m.pose = pose.pose.pose;
    m.pose.position.z = 0.01;
    m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
    marker_array.markers.push_back(m);

    // 2. Covariance Marker
    visualization_msgs::Marker cov;
    cov.header = pose.header;
    cov.ns = "estimation_covariance";
    cov.id = 1;
    cov.type = visualization_msgs::Marker::CYLINDER;
    cov.action = visualization_msgs::Marker::ADD;

    // Calculate position covariance ellipse orientation and scale using eigenvalue decomposition
    double p_cov_xx = pose.pose.covariance[0];
    double p_cov_xy = pose.pose.covariance[1];
    double p_cov_yy = pose.pose.covariance[7];

    double sum = p_cov_xx + p_cov_yy;
    double diff = p_cov_xx - p_cov_yy;
    double term = std::sqrt(diff * diff + 4.0 * p_cov_xy * p_cov_xy);
    double lambda1 = 0.5 * (sum + term);
    double lambda2 = 0.5 * (sum - term);
    double angle = 0.5 * std::atan2(2.0 * p_cov_xy, diff);

    cov.pose.position = pose.pose.pose.position;
    cov.pose.position.z = 0.005;

    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    cov.pose.orientation = tf2::toMsg(q);

    cov.color.r = 0.0; cov.color.g = 1.0; cov.color.b = 0.0; cov.color.a = 0.3;
    cov.scale.x = std::sqrt(std::max(0.001, lambda1)) * 2.0;
    cov.scale.y = std::sqrt(std::max(0.001, lambda2)) * 2.0;
    cov.scale.z = 0.01;
    marker_array.markers.push_back(cov);

    // 3. Vehicle Body & Label
    std::vector<double> color = {0.0, 1.0, 0.0, 0.7};
    _addVehicleMarker(marker_array, pose.pose.pose, "estimation", "Estimation", color);

    estimation_marker_pub_.publish(marker_array);

    // 4. Update Estimation Trajectory
    double dx = pose.pose.pose.position.x - last_published_pose_.position.x;
    double dy = pose.pose.pose.position.y - last_published_pose_.position.y;
    if (estimation_trajectory_.poses.empty() || (dx*dx + dy*dy) > 0.2*0.2) {
        geometry_msgs::PoseStamped ps;
        ps.header = pose.header;
        ps.pose = pose.pose.pose;
        estimation_trajectory_.header.stamp = pose.header.stamp;
        estimation_trajectory_.poses.push_back(ps);
        last_published_pose_ = ps.pose;

        if (estimation_trajectory_.poses.size() > 2000) estimation_trajectory_.poses.erase(estimation_trajectory_.poses.begin());
        estimation_trajectory_pub_.publish(estimation_trajectory_);
    }
}

void Visualizer::publishCorrection(const geometry_msgs::PoseWithCovarianceStamped& pose) {
    if (correction_marker_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray marker_array;

    // 1. Covariance Cylinder
    visualization_msgs::Marker cov;
    cov.header = pose.header;
    cov.ns = "correction_covariance";
    cov.id = 0;
    cov.type = visualization_msgs::Marker::CYLINDER;
    cov.action = visualization_msgs::Marker::ADD;

    // Calculate position covariance ellipse orientation and scale using eigenvalue decomposition
    double p_cov_xx = pose.pose.covariance[0];
    double p_cov_xy = pose.pose.covariance[1];
    double p_cov_yy = pose.pose.covariance[7];

    double sum = p_cov_xx + p_cov_yy;
    double diff = p_cov_xx - p_cov_yy;
    double term = std::sqrt(diff * diff + 4.0 * p_cov_xy * p_cov_xy);
    double lambda1 = 0.5 * (sum + term);
    double lambda2 = 0.5 * (sum - term);
    double angle = 0.5 * std::atan2(2.0 * p_cov_xy, diff);

    cov.pose.position = pose.pose.pose.position;
    cov.pose.position.z = 0.005;

    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    cov.pose.orientation = tf2::toMsg(q);

    cov.color.r = 1.0; cov.color.g = 0.0; cov.color.b = 0.0; cov.color.a = 0.3;
    cov.scale.x = std::sqrt(std::max(0.001, lambda1)) * 2.0;
    cov.scale.y = std::sqrt(std::max(0.001, lambda2)) * 2.0;
    cov.scale.z = 0.01;
    marker_array.markers.push_back(cov);

    // 2. Vehicle Body & Label
    std::vector<double> color = {1.0, 0.0, 0.0, 0.7};
    _addVehicleMarker(marker_array, pose.pose.pose, "correction", "Correction", color);

    correction_marker_pub_.publish(marker_array);
}

void Visualizer::clearCorrection() {
    if (correction_marker_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray marker_array;
    std::vector<std::string> namespaces = {"correction_covariance", "correction_body", "correction_label"};

    for (const auto& ns : namespaces) {
        visualization_msgs::Marker m;
        m.header.frame_id = global_frame_;
        m.header.stamp = ros::Time::now();
        m.ns = ns;
        m.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(m);
    }

    correction_marker_pub_.publish(marker_array);
}

void Visualizer::clearEstimation() {
    // 1. Clear estimation trajectory
    estimation_trajectory_.poses.clear();
    if (estimation_trajectory_pub_.getNumSubscribers() > 0) {
        estimation_trajectory_pub_.publish(estimation_trajectory_);
    }

    // 2. Clear estimation markers
    if (estimation_marker_pub_.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray marker_array;
        std::vector<std::string> namespaces = {"estimation_pose", "estimation_covariance", "estimation_body", "estimation_label"};
        for (const auto& ns : namespaces) {
            visualization_msgs::Marker m;
            m.header.frame_id = global_frame_;
            m.header.stamp = ros::Time::now();
            m.ns = ns;
            m.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(m);
        }
        estimation_marker_pub_.publish(marker_array);
    }
}

void Visualizer::clearObservation(const std::string& channel_name) {
    if (observation_pub_map_.find(channel_name) == observation_pub_map_.end()) return;
    auto& pub = observation_pub_map_[channel_name];
    if (pub.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray marker_array;
    std::vector<std::string> namespaces = {"observation_points", "observation_covariance"};
    for (const auto& ns : namespaces) {
        visualization_msgs::Marker m;
        m.header.frame_id = global_frame_;
        m.header.stamp = ros::Time::now();
        m.ns = ns;
        m.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(m);
    }
    pub.publish(marker_array);
}

void Visualizer::clearMapFeatures() {
    if (map_features_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray marker_array;
    std::vector<std::string> namespaces = {"map_lanes", "map_landmarks"};
    for (const auto& ns : namespaces) {
        visualization_msgs::Marker m;
        m.header.frame_id = global_frame_;
        m.header.stamp = ros::Time::now();
        m.ns = ns;
        m.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(m);
    }
    map_features_pub_.publish(marker_array);
}

void Visualizer::reset() {
    clearEstimation();
    clearCorrection();
    clearMapFeatures();
    for (const auto& pair : observation_pub_map_) {
        clearObservation(pair.first);
    }
}

void Visualizer::_addVehicleMarker(visualization_msgs::MarkerArray& marker_array,
                                    const geometry_msgs::Pose& pose,
                                    const std::string& ns,
                                    const std::string& label,
                                    const std::vector<double>& color) {
    double yaw = tf2::getYaw(pose.orientation);

    visualization_msgs::Marker body;
    body.header.frame_id = global_frame_;
    body.header.stamp = ros::Time::now();
    body.ns = ns + "_body";
    body.id = 0;
    body.type = visualization_msgs::Marker::CUBE;
    body.action = visualization_msgs::Marker::ADD;

    body.pose.position.x = pose.position.x + vehicle_length_offset_ * std::cos(yaw);
    body.pose.position.y = pose.position.y + vehicle_length_offset_ * std::sin(yaw);
    body.pose.position.z = vehicle_height_ / 2.0;
    body.pose.orientation = pose.orientation;

    body.scale.x = vehicle_length_;
    body.scale.y = vehicle_width_;
    body.scale.z = vehicle_height_;
    body.color.r = color[0]; body.color.g = color[1]; body.color.b = color[2]; body.color.a = color[3];
    marker_array.markers.push_back(body);

    visualization_msgs::Marker text;
    text.header = body.header;
    text.ns = ns + "_label";
    text.id = 1;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position = body.pose.position;
    text.pose.position.z = vehicle_height_ + 0.5;
    text.scale.z = 0.5;
    text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
    text.text = label;
    marker_array.markers.push_back(text);
}

void Visualizer::publishMapFeatures(const std::vector<MapFeature>& map_features) {
    if (map_features_pub_.getNumSubscribers() == 0 && !map_features_pub_.isLatched()) return;

    visualization_msgs::MarkerArray marker_array;

    // 1. Lane Points (Class 1) - SPHERE_LIST
    visualization_msgs::Marker lane_marker;
    lane_marker.header.frame_id = global_frame_;
    lane_marker.header.stamp = ros::Time::now();
    lane_marker.ns = "map_lanes";
    lane_marker.id = 0;
    lane_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    lane_marker.action = visualization_msgs::Marker::ADD;
    lane_marker.scale.x = resolution_;
    lane_marker.scale.y = resolution_;
    lane_marker.scale.z = 0.01;
    lane_marker.color.r = 1.0; lane_marker.color.g = 1.0; lane_marker.color.b = 1.0; lane_marker.color.a = 0.8; // White
    lane_marker.pose.orientation.w = 1.0;

    // 2. Landmark Points (Class 2) - SPHERE_LIST
    visualization_msgs::Marker landmark_marker;
    landmark_marker.header = lane_marker.header;
    landmark_marker.ns = "map_landmarks";
    landmark_marker.id = 1;
    landmark_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    landmark_marker.action = visualization_msgs::Marker::ADD;
    landmark_marker.scale.x = resolution_;
    landmark_marker.scale.y = resolution_;
    landmark_marker.scale.z = 0.01;
    landmark_marker.color.r = 1.0; landmark_marker.color.g = 0.3; landmark_marker.color.b = 0.3; landmark_marker.color.a = 0.8; // Coral Red
    landmark_marker.pose.orientation.w = 1.0;

    for (const auto& feat : map_features) {
        geometry_msgs::Point p;
        p.x = feat.x;
        p.y = feat.y;
        p.z = 0;
        if (feat.class_id == 1) {
            lane_marker.points.push_back(p);
        } else if (feat.class_id == 2) {
            landmark_marker.points.push_back(p);
        }
    }

    marker_array.markers.push_back(lane_marker);
    marker_array.markers.push_back(landmark_marker);
    map_features_pub_.publish(marker_array);
}

} // namespace carmaker_localization
