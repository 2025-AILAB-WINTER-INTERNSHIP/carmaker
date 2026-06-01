#ifndef CARMAKER_LOCALIZATION_ICP_MATCHER_H
#define CARMAKER_LOCALIZATION_ICP_MATCHER_H

#include "carmaker_localization/matcher_base.h"

namespace carmaker_localization {

class IcpMatcher : public MatcherBase {
public:
    IcpMatcher(double fitness_threshold, int max_iterations, double vision_base_std, double min_search_radius = 0.5);
    virtual ~IcpMatcher() = default;

    virtual MatchResult match(
        const std::vector<LocalFeature>& observed,
        const std::vector<MapFeature>& reference,
        const Eigen::Isometry2d& initial_guess) override;

private:
    double fitness_threshold_;
    int max_iterations_;
    double vision_base_std_;
    double min_search_radius_;
};

} // namespace carmaker_localization

#endif
