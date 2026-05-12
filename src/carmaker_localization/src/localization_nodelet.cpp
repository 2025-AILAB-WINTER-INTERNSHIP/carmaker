#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include "carmaker_localization/feature_extractor.h"
#include "carmaker_localization/ekf_core.h"
#include "carmaker_localization/visualizer.h"

namespace carmaker_localization {

class LocalizationNodelet : public nodelet::Nodelet {
public:
    LocalizationNodelet() = default;
    virtual ~LocalizationNodelet() = default;

private:
    virtual void onInit() override {
        NODELET_INFO("Initializing carmaker_localization Nodelet...");
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& pnh = getPrivateNodeHandle();

        // TODO: Initialize modules
    }
};

} // namespace carmaker_localization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(carmaker_localization::LocalizationNodelet, nodelet::Nodelet)
