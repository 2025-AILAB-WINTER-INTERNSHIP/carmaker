#ifndef CARMAKER_LOCALIZATION_ICP_REGISTRATION_H
#define CARMAKER_LOCALIZATION_ICP_REGISTRATION_H

#include "carmaker_localization/registration_base.h"

namespace carmaker_localization {

struct IcpParams {
    double fitness_threshold = 0.35;
    int max_iterations = 50;
    double vision_base_std = 0.1;
    double vision_base_yaw_std = 0.02;
    double min_search_radius = 0.5;
    double max_covariance = 10.0;
    int max_observed_features = 400;
};

class IcpRegistration : public RegistrationBase {
public:
    explicit IcpRegistration(const IcpParams& params);
    virtual ~IcpRegistration() = default;

    virtual RegistrationResult align(
        const std::vector<LocalFeature>& observed,
        const std::vector<LandmarkFeature>& landmarks,
        const Eigen::Isometry2d& initial_guess,
        bool collect_debug_info = false,
        bool collect_associations = false,
        bool collect_iteration_trace = false) override;

private:
    double fitness_threshold_;
    int max_iterations_;
    double vision_base_std_;
    double vision_base_yaw_std_;
    double min_search_radius_;
    double max_covariance_; ///< Eigen-capping 최대 분산 상한값 [m^2 or rad^2]
    int max_observed_features_;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_ICP_REGISTRATION_H
