#include "carmaker_localization/evaluation_utils.h"

#include <ros/package.h>
#include <unistd.h>

namespace carmaker_localization {

std::string resolvePackagePath(const std::string& path) {
    if (path.empty() || path.front() == '/') return path;

    std::string pkg_path = ros::package::getPath("carmaker_localization");
    if (pkg_path.empty()) return path;

    // Support install space to source space fallback for evaluation output path
    const size_t install_idx = pkg_path.find("/install/");
    if (install_idx != std::string::npos) {
        const std::string src_path = pkg_path.substr(0, install_idx) + "/src/carmaker_localization";
        if (access(src_path.c_str(), F_OK) == 0) {
            pkg_path = src_path;
        }
    }
    return pkg_path + "/" + path;
}

void writeCsvHeader(std::ostream& csv) {
    csv << "grid_i,grid_j,cell_x,cell_y,mean_observed_count,landmark_count,"
        << "attempts,success_count,success_rate,mean_fitness,mean_iterations,mean_latency_ms,"
        << "longitudinal_rmse,lateral_rmse,yaw_rmse,"
        << "cov_xx,cov_xy,cov_xyaw,cov_yx,cov_yy,cov_yyaw,cov_yawx,cov_yawy,cov_yawyaw,cov_trace,cov_determinant\n";
}

void writeCsvRow(std::ostream& csv, const EvaluationRow& row) {
    csv << row.grid.grid_i << "," << row.grid.grid_j << "," << row.grid.cell_x << "," << row.grid.cell_y << ","
        << row.mean_observed_count << "," << row.landmark_count << ","
        << row.attempts << "," << row.success_count << "," << row.success_rate << ","
        << row.mean_fitness << "," << row.mean_iterations << "," << row.mean_latency_ms << ","
        << row.longitudinal_rmse << "," << row.lateral_rmse << "," << row.yaw_rmse << ","
        << row.cov_xx << "," << row.cov_xy << "," << row.cov_xyaw << ","
        << row.cov_yx << "," << row.cov_yy << "," << row.cov_yyaw << ","
        << row.cov_yawx << "," << row.cov_yawy << "," << row.cov_yawyaw << ","
        << row.cov_trace << "," << row.cov_determinant << "\n";
}

} // namespace carmaker_localization
