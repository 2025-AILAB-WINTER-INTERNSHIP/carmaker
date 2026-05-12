#ifndef CARMAKER_LOCALIZATION_MATCHER_BASE_H
#define CARMAKER_LOCALIZATION_MATCHER_BASE_H

#include <vector>
#include <Eigen/Geometry>
#include <carmaker_msgs/LocalFeature.h>
#include "carmaker_localization/map_loader_base.h"

namespace carmaker_localization {

struct MatchResult {
    bool success;
    Eigen::Isometry2d transform; // Correction transform (dx, dy, dyaw)
    Eigen::Matrix3d covariance;  // Matching uncertainty
    double fitness_score;        // 0.0 ~ 1.0
};

/**
 * @brief Abstract interface for matching algorithms.
 * Supports ICP, NDT, Point-to-Line, etc.
 */
class MatcherBase {
public:
    virtual ~MatcherBase() = default;

    /**
     * @brief Match observed features with reference map features
     * @param observed Features from sensors (in base_link)
     * @param reference Features from map (in map/world)
     * @param initial_guess Initial pose of base_link in map
     * @return Result of matching
     */
    virtual MatchResult match(
        const std::vector<carmaker_msgs::LocalFeature>& observed,
        const std::vector<MapFeature>& reference,
        const Eigen::Isometry2d& initial_guess) = 0;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_MATCHER_BASE_H
