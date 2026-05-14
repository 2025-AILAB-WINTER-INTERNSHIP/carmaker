#ifndef CARMAKER_LOCALIZATION_VISUALIZER_H
#define CARMAKER_LOCALIZATION_VISUALIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <carmaker_msgs/LocalFeatures.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <carmaker_msgs/DynamicsInfo.h>
#include <opencv2/opencv.hpp>
#include <map>
#include <string>
#include <vector>

namespace carmaker_localization {

class Visualizer {
public:
    explicit Visualizer(const ros::NodeHandle& nh);
    ~Visualizer() = default;

    void publishEstimation(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void publishCorrection(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void publishObservation(const std::string& channel_name, const carmaker_msgs::LocalFeatures& features);
    void publishSvmImage(const cv::Mat& svm_image);

private:
    ros::NodeHandle nh_;
    ros::Publisher svm_pub_;
    ros::Publisher feature_pc_pub_;
    ros::Publisher estimation_marker_pub_;
    ros::Publisher correction_marker_pub_;
    ros::Publisher estimation_trajectory_pub_;
    std::map<std::string, ros::Publisher> observation_pub_map_;

    nav_msgs::Path estimation_trajectory_;
    geometry_msgs::Pose last_published_pose_;

    double resolution_;
    double vehicle_length_, vehicle_width_, vehicle_height_, vehicle_length_offset_;
    std::string global_frame_;
    std::string prediction_frame_;

    void addVehicleMarker(visualization_msgs::MarkerArray& marker_array,
                           const geometry_msgs::Pose& pose,
                           const std::string& ns,
                           const std::string& label,
                           const std::vector<double>& color);
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_VISUALIZER_H
