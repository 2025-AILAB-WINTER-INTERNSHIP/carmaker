#ifndef CARMAKER_LOCALIZATION_REGISTRATION_BASE_H
#define CARMAKER_LOCALIZATION_REGISTRATION_BASE_H

#include <vector>
#include <Eigen/Geometry>
#include "carmaker_localization/feature_extractor.h"
#include "carmaker_localization/feature_loader_base.h"

namespace carmaker_localization {

struct RegistrationResult {
    bool success;
    Eigen::Isometry2d transform; // Correction transform (dx, dy, dyaw)
    Eigen::Matrix3d covariance;  // Registration uncertainty
    double fitness_score;        // 0.0 ~ 1.0
    size_t num_observed;         // Number of features observed
    size_t num_reference;        // Number of reference features used
};

/**
 * @brief Abstract interface for feature registration algorithms.
 * Supports ICP, NDT, Point-to-Line, etc.
 */
class RegistrationBase {
public:
    virtual ~RegistrationBase() = default;

    /**
     * @brief Align observed features with reference features
     * @param observed Features from sensors (in Fr1A)
     * @param reference Reference features from map (in global frame)
     * @param initial_guess Initial pose guess of Fr1A in global frame
     * @return Result of registration alignment
     */
    virtual RegistrationResult align(
        const std::vector<LocalFeature>& observed,
        const std::vector<ReferenceFeature>& reference,
        const Eigen::Isometry2d& initial_guess) = 0;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_REGISTRATION_BASE_H
