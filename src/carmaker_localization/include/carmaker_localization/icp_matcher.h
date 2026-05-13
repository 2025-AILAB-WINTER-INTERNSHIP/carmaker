#ifndef CARMAKER_LOCALIZATION_ICP_MATCHER_H
#define CARMAKER_LOCALIZATION_ICP_MATCHER_H

#include "carmaker_localization/matcher_base.h"
#include <opencv2/flann.hpp>

namespace carmaker_localization {

class IcpMatcher : public MatcherBase {
public:
    IcpMatcher(double fitness_threshold, int max_iterations, double vision_base_std);
    virtual ~IcpMatcher() = default;

    virtual MatchResult match(
        const std::vector<carmaker_msgs::LocalFeature>& observed,
        const std::vector<MapFeature>& reference,
        const Eigen::Isometry2d& initial_guess) override;

private:
    double fitness_threshold_;
    int max_iterations_;
    double vision_base_std_;
};

} // namespace carmaker_localization

#endif
