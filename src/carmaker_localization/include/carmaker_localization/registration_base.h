#ifndef CARMAKER_LOCALIZATION_REGISTRATION_BASE_H
#define CARMAKER_LOCALIZATION_REGISTRATION_BASE_H

#include <optional>
#include <vector>
#include <Eigen/Geometry>
#include "carmaker_localization/feature_extractor.h"
#include "carmaker_localization/landmark_loader_base.h"

namespace carmaker_localization {

struct Association {
    double obs_x;            // Raw sensor feature X (in Fr1A/bumper frame)
    double obs_y;            // Raw sensor feature Y (in Fr1A/bumper frame)
    double landmark_x;       // Associated landmark X (in global frame)
    double landmark_y;       // Associated landmark Y (in global frame)
    double dist_mahalanobis; // Mahalanobis matching distance
    uint8_t class_id = 0;    // Feature class id
    bool is_inlier;          // True if within the inlier threshold
};

struct IterationTrace {
    int iteration = 0;
    double estimated_x = 0.0;
    double estimated_y = 0.0;
    double estimated_yaw = 0.0;
    double elapsed_ms = 0.0;
    std::vector<Association> associations;
};

struct RegistrationDebugInfo {
    std::vector<LandmarkFeature> query_landmarks;
    std::vector<Association> associations;
    std::vector<IterationTrace> iteration_traces;
    int iterations = 0;
    double latency_ms = 0.0;
};

struct RegistrationResult {
    bool success = false;
    Eigen::Isometry2d transform = Eigen::Isometry2d::Identity(); // Estimated Fr1A pose in map frame
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();    // Registration uncertainty
    double fitness_score = 0.0;                                  // 0.0 ~ 1.0
    size_t num_observed = 0;                                     // Number of features observed
    size_t num_landmarks = 0;                                    // Number of landmarks used
    std::optional<RegistrationDebugInfo> debug;
};

/**
 * @brief Abstract interface for feature registration algorithms.
 * Supports ICP, NDT, Point-to-Line, etc.
 */
class RegistrationBase {
public:
    virtual ~RegistrationBase() = default;

    /**
     * @brief Align observed features with landmarks
     * @param observed Features from sensors (in Fr1A)
     * @param landmarks Landmarks in global frame
     * @param initial_guess Initial pose guess of Fr1A in global frame
     * @return Result of registration alignment
     */
    virtual RegistrationResult align(
        const std::vector<LocalFeature>& observed,
        const std::vector<LandmarkFeature>& landmarks,
        const Eigen::Isometry2d& initial_guess,
        bool collect_debug_info = false,
        bool collect_associations = false,
        bool collect_iteration_trace = false) = 0;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_REGISTRATION_BASE_H
