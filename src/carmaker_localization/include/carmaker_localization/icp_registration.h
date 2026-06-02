#ifndef CARMAKER_LOCALIZATION_ICP_REGISTRATION_H
#define CARMAKER_LOCALIZATION_ICP_REGISTRATION_H

#include "carmaker_localization/registration_base.h"

namespace carmaker_localization {

class IcpRegistration : public RegistrationBase {
public:
    IcpRegistration(double fitness_threshold, int max_iterations, double vision_base_std, double min_search_radius = 0.5);
    virtual ~IcpRegistration() = default;

    virtual RegistrationResult align(
        const std::vector<LocalFeature>& observed,
        const std::vector<ReferenceFeature>& reference,
        const Eigen::Isometry2d& initial_guess) override;

private:
    double fitness_threshold_;
    int max_iterations_;
    double vision_base_std_;
    double min_search_radius_;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_ICP_REGISTRATION_H
