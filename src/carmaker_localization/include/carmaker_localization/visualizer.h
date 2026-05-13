#ifndef CARMAKER_LOCALIZATION_VISUALIZER_H
#define CARMAKER_LOCALIZATION_VISUALIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <carmaker_msgs/LocalFeatures.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/opencv.hpp>

namespace carmaker_localization {

class Visualizer {
public:
    explicit Visualizer(const ros::NodeHandle& nh);
    ~Visualizer() = default;

    void publishSvmImage(const cv::Mat& svm_image);
    void publishFeatures(const carmaker_msgs::LocalFeatures& features);
    void publishEkfState(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void publishMapCorrection(const geometry_msgs::PoseWithCovarianceStamped& pose);

private:
    ros::NodeHandle nh_;
    ros::Publisher svm_pub_;
    ros::Publisher feature_marker_pub_;
    ros::Publisher ekf_marker_pub_;
    ros::Publisher map_match_marker_pub_;
    double resolution_;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_VISUALIZER_H
