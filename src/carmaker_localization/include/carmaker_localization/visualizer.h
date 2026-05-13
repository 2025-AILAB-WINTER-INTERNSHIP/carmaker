#ifndef CARMAKER_LOCALIZATION_VISUALIZER_H
#define CARMAKER_LOCALIZATION_VISUALIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <carmaker_msgs/LocalFeatures.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <carmaker_msgs/DynamicsInfo.h>
#include <opencv2/opencv.hpp>

namespace carmaker_localization {

class Visualizer {
public:
    explicit Visualizer(const ros::NodeHandle& nh);
    ~Visualizer() = default;

    void publishSvmImage(const cv::Mat& svm_image);
    void publishFeatures(const carmaker_msgs::LocalFeatures& features);
    void publishEstimatedState(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void publishCorrectionState(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void publishGtPose(const carmaker_msgs::DynamicsInfo& gt);

private:
    ros::NodeHandle nh_;
    ros::Publisher svm_pub_;
    ros::Publisher feature_marker_pub_;
    ros::Publisher est_marker_pub_;
    ros::Publisher obs_marker_pub_;
    ros::Publisher gt_path_pub_;
    ros::Publisher pred_path_pub_;

    nav_msgs::Path gt_path_;
    nav_msgs::Path pred_path_;
    geometry_msgs::Pose last_gt_pose_;
    geometry_msgs::Pose last_pred_pose_;

    double resolution_;
    double length_, width_, front_edge_, rear_edge_;
    std::string global_frame_;
    std::string prediction_frame_;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_VISUALIZER_H
