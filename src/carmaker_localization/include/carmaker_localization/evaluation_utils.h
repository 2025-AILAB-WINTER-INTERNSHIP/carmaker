#ifndef CARMAKER_LOCALIZATION_EVALUATION_UTILS_H
#define CARMAKER_LOCALIZATION_EVALUATION_UTILS_H

#include <string>
#include <vector>
#include <limits>
#include <ostream>
#include <Eigen/Core>

namespace carmaker_localization {

constexpr double kSuccessPositionErrorMeters = 0.15;
const double kNoData = std::numeric_limits<double>::quiet_NaN();

struct GridCell {
    int grid_i = 0;
    int grid_j = 0;
    double cell_x = 0.0;
    double cell_y = 0.0;
};

struct EvaluationRow {
    GridCell grid;
    double mean_observed_count = kNoData;
    double landmark_count = kNoData;
    double attempts = 0.0;
    double success_count = 0.0;
    double success_rate = 0.0;
    double mean_fitness = kNoData;
    double mean_iterations = kNoData;
    double mean_latency_ms = kNoData;
    double longitudinal_rmse = kNoData;
    double lateral_rmse = kNoData;
    double yaw_rmse = kNoData;
    double cov_xx = kNoData;
    double cov_xy = kNoData;
    double cov_xyaw = kNoData;
    double cov_yx = kNoData;
    double cov_yy = kNoData;
    double cov_yyaw = kNoData;
    double cov_yawx = kNoData;
    double cov_yawy = kNoData;
    double cov_yawyaw = kNoData;
    double cov_trace = kNoData;
    double cov_determinant = kNoData;
    bool has_data = false;
};

std::string resolvePackagePath(const std::string& path);
void writeCsvHeader(std::ostream& csv);
void writeCsvRow(std::ostream& csv, const EvaluationRow& row);

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_EVALUATION_UTILS_H
