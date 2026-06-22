#include <ros/ros.h>
#include <ros/package.h>

#include "carmaker_localization/icp_registration.h"
#include "carmaker_localization/osm_landmark_loader.h"
#include "carmaker_localization/evaluation_utils.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cctype>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <omp.h>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

using namespace carmaker_localization;

namespace {

struct DebugPose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    std::string label;
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_yaw = 0.0;
};

struct EvaluatorConfig {
    std::string landmark_file;
    double center_x = 0.0;
    double center_y = -4.33;
    double grid_step = 1.0;
    int num_yaw_angles = 4;
    double sensor_range = 15.0;
    double rear_axle_offset_x = 0.82;
    double fitness_threshold = 0.35;
    int max_iterations = 50;
    double min_search_radius = 5.0;
    double max_covariance = 10.0;
    double vision_base_std = 0.1;
    double vision_base_yaw_std = 0.1;
    int max_observed_features = 400;
    std::string output_csv_path = "grid_registration_results.csv";
    std::string output_dir = "docs";
    std::string landmarks_csv_path = "landmarks.csv";
    std::vector<DebugPose> debug_poses;
};

template <typename T>
void getParamFallback(const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
                      const std::string& private_key, const std::string& global_key, T& value) {
    if (pnh.getParam(private_key, value)) return;
    if (pnh.getParam(global_key, value)) return;
    nh.getParam(global_key, value);
}

std::string sanitizeLabel(const std::string& label, size_t index) {
    std::string out;
    out.reserve(label.size());
    for (char ch : label) {
        if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '-') {
            out.push_back(ch);
        } else if (!out.empty() && out.back() != '_') {
            out.push_back('_');
        }
    }
    if (out.empty()) {
        out = "pose_" + std::to_string(index);
    }
    return out;
}

std::string debugPoseFileName(size_t index) {
    std::ostringstream oss;
    oss << "icp_debug_" << std::setw(3) << std::setfill('0') << (index + 1) << ".json";
    return oss.str();
}

bool getXmlRpcDouble(XmlRpc::XmlRpcValue& value, const std::string& key, double& out) {
    if (!value.hasMember(key)) return false;
    auto& item = value[key];
    if (item.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        out = static_cast<int>(item);
        return true;
    }
    if (item.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        out = static_cast<double>(item);
        return true;
    }
    return false;
}

void loadDebugPosesFromValue(XmlRpc::XmlRpcValue& poses, std::vector<DebugPose>& out) {
    if (poses.getType() != XmlRpc::XmlRpcValue::TypeArray) return;

    out.clear();
    out.reserve(static_cast<size_t>(poses.size()));
    for (int i = 0; i < poses.size(); ++i) {
        auto& pose_value = poses[i];
        if (pose_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;

        DebugPose pose;
        if (!getXmlRpcDouble(pose_value, "x", pose.x) ||
            !getXmlRpcDouble(pose_value, "y", pose.y)) {
            continue;
        }

        double yaw_deg = 0.0;
        if (getXmlRpcDouble(pose_value, "yaw_deg", yaw_deg)) {
            pose.yaw = yaw_deg * M_PI / 180.0;
        } else {
            getXmlRpcDouble(pose_value, "yaw", pose.yaw);
        }

        if (pose_value.hasMember("label") &&
            pose_value["label"].getType() == XmlRpc::XmlRpcValue::TypeString) {
            pose.label = sanitizeLabel(static_cast<std::string>(pose_value["label"]), out.size());
        } else {
            pose.label = sanitizeLabel("", out.size());
        }

        getXmlRpcDouble(pose_value, "offset_x", pose.offset_x);
        getXmlRpcDouble(pose_value, "offset_y", pose.offset_y);
        double offset_yaw_deg = 0.0;
        if (getXmlRpcDouble(pose_value, "offset_yaw_deg", offset_yaw_deg)) {
            pose.offset_yaw = offset_yaw_deg * M_PI / 180.0;
        } else {
            getXmlRpcDouble(pose_value, "offset_yaw", pose.offset_yaw);
        }

        out.push_back(pose);
    }
}

bool loadConfig(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, EvaluatorConfig& cfg) {
    if (!pnh.getParam("landmark_file", cfg.landmark_file) &&
        !pnh.getParam("evaluator/landmark_file", cfg.landmark_file) &&
        !nh.getParam("evaluator/landmark_file", cfg.landmark_file) &&
        !pnh.getParam("map_file", cfg.landmark_file) &&
        !pnh.getParam("evaluator/map_file", cfg.landmark_file) &&
        !nh.getParam("evaluator/map_file", cfg.landmark_file)) {
        return false;
    }

    getParamFallback(nh, pnh, "center_x", "evaluator/center_x", cfg.center_x);
    getParamFallback(nh, pnh, "center_y", "evaluator/center_y", cfg.center_y);
    getParamFallback(nh, pnh, "grid_step", "evaluator/grid_step", cfg.grid_step);
    getParamFallback(nh, pnh, "num_yaw_angles", "evaluator/static/num_yaw_angles", cfg.num_yaw_angles);
    getParamFallback(nh, pnh, "sensor_range", "evaluator/static/sensor_range", cfg.sensor_range);
    getParamFallback(nh, pnh, "rear_axle_offset_x", "vehicle/rear_axle_offset_x", cfg.rear_axle_offset_x);
    getParamFallback(nh, pnh, "fitness_threshold", "feature_registration/fitness_threshold", cfg.fitness_threshold);
    getParamFallback(nh, pnh, "max_iterations", "feature_registration/max_iterations", cfg.max_iterations);
    getParamFallback(nh, pnh, "min_search_radius", "feature_registration/min_search_radius", cfg.min_search_radius);
    getParamFallback(nh, pnh, "max_covariance", "feature_registration/max_covariance", cfg.max_covariance);
    getParamFallback(nh, pnh, "max_observed_features", "feature_registration/max_observed_features", cfg.max_observed_features);
    getParamFallback(nh, pnh, "vision_base_std", "ekf/camera/base_std", cfg.vision_base_std);
    getParamFallback(nh, pnh, "vision_base_yaw_std", "ekf/camera/base_yaw_std", cfg.vision_base_yaw_std);
    getParamFallback(nh, pnh, "output_csv_path", "evaluator/output_csv_path", cfg.output_csv_path);
    getParamFallback(nh, pnh, "output_dir", "evaluator/output_dir", cfg.output_dir);
    getParamFallback(nh, pnh, "landmarks_csv_path", "evaluator/landmarks_csv_path", cfg.landmarks_csv_path);
    if (cfg.landmarks_csv_path == "landmarks.csv") {
        getParamFallback(nh, pnh, "map_features_csv_path", "evaluator/map_features_csv_path", cfg.landmarks_csv_path);
    }

    XmlRpc::XmlRpcValue debug_poses;
    if (pnh.getParam("debug_poses", debug_poses) ||
        pnh.getParam("static/debug_poses", debug_poses) ||
        pnh.getParam("evaluator/static/debug_poses", debug_poses) ||
        nh.getParam("evaluator/static/debug_poses", debug_poses)) {
        loadDebugPosesFromValue(debug_poses, cfg.debug_poses);
    }

    cfg.grid_step = std::max(1e-3, cfg.grid_step);
    cfg.num_yaw_angles = std::max(1, cfg.num_yaw_angles);
    cfg.landmark_file = resolvePackagePath(cfg.landmark_file);
    cfg.output_csv_path = resolvePackagePath(cfg.output_csv_path);
    cfg.output_dir = resolvePackagePath(cfg.output_dir);
    cfg.landmarks_csv_path = resolvePackagePath(cfg.landmarks_csv_path);
    return true;
}

GridCell toGridPoint(double x, double y, const EvaluatorConfig& cfg) {
    GridCell point;
    point.grid_i = static_cast<int>(std::llround((x - cfg.center_x) / cfg.grid_step));
    point.grid_j = static_cast<int>(std::llround((y - cfg.center_y) / cfg.grid_step));
    point.cell_x = cfg.center_x + point.grid_i * cfg.grid_step;
    point.cell_y = cfg.center_y + point.grid_j * cfg.grid_step;
    return point;
}

std::vector<GridCell> generateGridPoints(const nav_msgs::OccupancyGrid& grid, const EvaluatorConfig& cfg) {
    const double res = grid.info.resolution;
    const double origin_x = grid.info.origin.position.x;
    const double origin_y = grid.info.origin.position.y;
    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);
    const double max_x = origin_x + width * res;
    const double max_y = origin_y + height * res;

    const int i_min = static_cast<int>(std::floor((origin_x - cfg.center_x) / cfg.grid_step));
    const int i_max = static_cast<int>(std::ceil((max_x - cfg.center_x) / cfg.grid_step));
    const int j_min = static_cast<int>(std::floor((origin_y - cfg.center_y) / cfg.grid_step));
    const int j_max = static_cast<int>(std::ceil((max_y - cfg.center_y) / cfg.grid_step));

    std::vector<GridCell> points;
    points.reserve(static_cast<size_t>(std::max(0, i_max - i_min + 1)) * std::max(0, j_max - j_min + 1));
    for (int j = j_min; j <= j_max; ++j) {
        for (int i = i_min; i <= i_max; ++i) {
            const double cell_x = cfg.center_x + i * cfg.grid_step;
            const double cell_y = cfg.center_y + j * cfg.grid_step;
            const int ix = static_cast<int>(std::round((cell_x - origin_x) / res));
            const int iy = static_cast<int>(std::round((cell_y - origin_y) / res));
            if (ix < 0 || ix >= width || iy < 0 || iy >= height) continue;

            const size_t idx = static_cast<size_t>(iy) * static_cast<size_t>(width) + static_cast<size_t>(ix);
            if (grid.data[idx] == 0) points.push_back({i, j, cell_x, cell_y});
        }
    }
    return points;
}

double rmse(double sum_sq, double count) {
    return count > 0.0 ? std::sqrt(sum_sq / count) : kNoData;
}



std::vector<LocalFeature> createVisibleFeatures(const std::vector<LandmarkFeature>& landmarks,
                                                const Eigen::Isometry2d& T_map_bumper_true,
                                                double sensor_range) {
    std::vector<LocalFeature> observed;
    observed.reserve(landmarks.size());
    for (const auto& landmark : landmarks) {
        const Eigen::Vector2d pt_bumper = T_map_bumper_true.inverse() * Eigen::Vector2d(landmark.x, landmark.y);
        if (pt_bumper.norm() > sensor_range) continue;

        LocalFeature obs;
        obs.x = pt_bumper.x();
        obs.y = pt_bumper.y();
        obs.cov_xx = 0.0;
        obs.cov_yy = 0.0;
        obs.cov_xy = 0.0;
        obs.class_id = landmark.class_id;
        observed.push_back(obs);
    }
    return observed;
}

bool writeLandmarksCsv(const std::string& path, const std::vector<LandmarkFeature>& landmarks) {
    std::ofstream csv(path);
    if (!csv.is_open()) return false;

    csv << "x,y,class_id\n";
    for (const auto& landmark : landmarks) {
        csv << landmark.x << "," << landmark.y << "," << static_cast<int>(landmark.class_id) << "\n";
    }
    return true;
}

bool ensureDirectory(const std::string& path) {
    if (path.empty()) return false;

    struct stat info;
    if (stat(path.c_str(), &info) == 0) {
        return S_ISDIR(info.st_mode);
    }
    return mkdir(path.c_str(), 0755) == 0;
}

void writeJsonString(std::ostream& os, const std::string& value) {
    os << "\"";
    for (char ch : value) {
        if (ch == '\\' || ch == '"') {
            os << "\\" << ch;
        } else if (ch == '\n') {
            os << "\\n";
        } else {
            os << ch;
        }
    }
    os << "\"";
}

void writeJsonFeatureArray(std::ostream& os, const std::vector<LandmarkFeature>& landmarks) {
    os << "[";
    for (size_t i = 0; i < landmarks.size(); ++i) {
        if (i > 0) os << ",";
        os << "{\"x\":" << landmarks[i].x
           << ",\"y\":" << landmarks[i].y
           << ",\"c\":" << static_cast<int>(landmarks[i].class_id) << "}";
    }
    os << "]";
}

void writeJsonLocalFeatureArray(std::ostream& os, const std::vector<LocalFeature>& features) {
    os << "[";
    for (size_t i = 0; i < features.size(); ++i) {
        if (i > 0) os << ",";
        os << "{\"x\":" << features[i].x
           << ",\"y\":" << features[i].y
           << ",\"c\":" << static_cast<int>(features[i].class_id) << "}";
    }
    os << "]";
}

void writeJsonTraceArray(std::ostream& os, const RegistrationResult& result) {
    os << "[";
    if (result.debug) {
        const auto& traces = result.debug->iteration_traces;
        for (size_t i = 0; i < traces.size(); ++i) {
            const auto& trace = traces[i];
            if (i > 0) os << ",";
            os << "{\"iteration\":" << trace.iteration
               << ",\"x\":" << trace.estimated_x
               << ",\"y\":" << trace.estimated_y
               << ",\"yaw\":" << trace.estimated_yaw
               << ",\"elapsed_ms\":" << trace.elapsed_ms
               << ",\"associations\":[";

            for (size_t j = 0; j < trace.associations.size(); ++j) {
                const auto& assoc = trace.associations[j];
                if (j > 0) os << ",";
                os << "{\"ox\":" << assoc.obs_x
                   << ",\"oy\":" << assoc.obs_y
                   << ",\"lx\":" << assoc.landmark_x
                   << ",\"ly\":" << assoc.landmark_y
                   << ",\"d\":" << assoc.dist_mahalanobis
                   << ",\"c\":" << static_cast<int>(assoc.class_id)
                   << ",\"is_inlier\":" << (assoc.is_inlier ? "true" : "false") << "}";
            }
            os << "]}";
        }
    }
    os << "]";
}

bool writeDebugJson(const std::string& path,
                    const DebugPose& pose,
                    double rear_axle_offset_x,
                    double sensor_range,
                    const std::vector<LandmarkFeature>& landmarks,
                    const std::vector<LocalFeature>& observed,
                    const RegistrationResult& result,
                    const std::vector<LocalFeature>& moving_landmarks,
                    const RegistrationResult& landmark_to_landmark_result) {
    std::ofstream json(path);
    if (!json.is_open()) return false;

    json << std::setprecision(10);
    json << "{";
    json << "\"label\":";
    writeJsonString(json, pose.label);
    json << ",\"gt\":{\"x\":" << pose.x
         << ",\"y\":" << pose.y
         << ",\"yaw\":" << pose.yaw
         << ",\"yaw_deg\":" << pose.yaw * 180.0 / M_PI << "}";
    json << ",\"initial_offset\":{\"x\":" << pose.offset_x
         << ",\"y\":" << pose.offset_y
         << ",\"yaw\":" << pose.offset_yaw
         << ",\"yaw_deg\":" << pose.offset_yaw * 180.0 / M_PI << "}";
    json << ",\"rear_offset\":" << rear_axle_offset_x;
    json << ",\"sensor_range\":" << sensor_range;
    json << ",\"result\":{\"success\":" << (result.success ? "true" : "false")
         << ",\"fitness\":" << result.fitness_score
         << ",\"iterations\":" << (result.debug ? result.debug->iterations : 0)
         << ",\"latency_ms\":" << (result.debug ? result.debug->latency_ms : 0.0)
         << ",\"observed_count\":" << result.num_observed
         << ",\"landmark_count\":" << result.num_landmarks
         << ",\"cov_xx\":" << result.covariance(0,0)
         << ",\"cov_yy\":" << result.covariance(1,1)
         << ",\"cov_yawyaw\":" << result.covariance(2,2)
         << ",\"cov_trace\":" << result.covariance.trace()
         << ",\"cov_det\":" << result.covariance.determinant() << "}";
    json << ",\"landmark_to_landmark_result\":{\"success\":" << (landmark_to_landmark_result.success ? "true" : "false")
         << ",\"fitness\":" << landmark_to_landmark_result.fitness_score
         << ",\"iterations\":" << (landmark_to_landmark_result.debug ? landmark_to_landmark_result.debug->iterations : 0)
         << ",\"latency_ms\":" << (landmark_to_landmark_result.debug ? landmark_to_landmark_result.debug->latency_ms : 0.0)
         << ",\"observed_count\":" << landmark_to_landmark_result.num_observed
         << ",\"landmark_count\":" << landmark_to_landmark_result.num_landmarks
         << ",\"cov_xx\":" << landmark_to_landmark_result.covariance(0,0)
         << ",\"cov_yy\":" << landmark_to_landmark_result.covariance(1,1)
         << ",\"cov_yawyaw\":" << landmark_to_landmark_result.covariance(2,2)
         << ",\"cov_trace\":" << landmark_to_landmark_result.covariance.trace()
         << ",\"cov_det\":" << landmark_to_landmark_result.covariance.determinant() << "}";
    json << ",\"landmarks\":";
    writeJsonFeatureArray(json, landmarks);
    json << ",\"observed_features\":";
    writeJsonLocalFeatureArray(json, observed);
    json << ",\"moving_landmarks\":";
    writeJsonLocalFeatureArray(json, moving_landmarks);
    json << ",\"traces\":";
    writeJsonTraceArray(json, result);
    json << ",\"landmark_traces\":";
    writeJsonTraceArray(json, landmark_to_landmark_result);
    json << "}\n";
    return true;
}

EvaluationRow evaluatePoint(const GridCell& point, const EvaluatorConfig& cfg,
                            const std::vector<LandmarkFeature>& landmarks) {
    EvaluationRow row;
    row.grid = point;
    if (landmarks.empty()) return row;

    IcpParams icp_params;
    icp_params.fitness_threshold = cfg.fitness_threshold;
    icp_params.max_iterations = cfg.max_iterations;
    icp_params.vision_base_std = cfg.vision_base_std;
    icp_params.vision_base_yaw_std = cfg.vision_base_yaw_std;
    icp_params.min_search_radius = cfg.min_search_radius;
    icp_params.max_covariance = cfg.max_covariance;
    icp_params.max_observed_features = cfg.max_observed_features;

    IcpRegistration icp(icp_params);

    Eigen::Isometry2d T_rear_bumper = Eigen::Isometry2d::Identity();
    T_rear_bumper.translation() = Eigen::Vector2d(-cfg.rear_axle_offset_x, 0.0);

    row.landmark_count = static_cast<double>(landmarks.size());

    double sum_observed_count = 0.0;
    double sum_fitness = 0.0;
    double sum_iterations = 0.0;
    double sum_latency = 0.0;
    double sum_long_sq = 0.0;
    double sum_lat_sq = 0.0;
    double sum_yaw_sq = 0.0;
    Eigen::Matrix3d sum_cov = Eigen::Matrix3d::Zero();

    for (int yaw_idx = 0; yaw_idx < cfg.num_yaw_angles; ++yaw_idx) {
        const double true_yaw = yaw_idx * (2.0 * M_PI / static_cast<double>(cfg.num_yaw_angles));
        Eigen::Isometry2d T_map_rear_true = Eigen::Isometry2d::Identity();
        T_map_rear_true.translation() = Eigen::Vector2d(point.cell_x, point.cell_y);
        T_map_rear_true.linear() = Eigen::Rotation2Dd(true_yaw).toRotationMatrix();
        const Eigen::Isometry2d T_map_bumper_true = T_map_rear_true * T_rear_bumper;

        row.attempts += 1.0;

        std::vector<LocalFeature> observed = createVisibleFeatures(landmarks, T_map_bumper_true, cfg.sensor_range);
        sum_observed_count += static_cast<double>(observed.size());

        RegistrationResult result = icp.align(observed, landmarks, T_map_bumper_true, true);
        const int iterations = result.debug ? result.debug->iterations : 0;
        const double latency_ms = result.debug ? result.debug->latency_ms : kNoData;

        if (!result.success) continue;

        const Eigen::Isometry2d T_map_rear_est = result.transform * T_rear_bumper.inverse();
        const double dx = T_map_rear_est.translation().x() - point.cell_x;
        const double dy = T_map_rear_est.translation().y() - point.cell_y;
        double dyaw = Eigen::Rotation2Dd(T_map_rear_est.linear()).angle() - true_yaw;
        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

        const double longitudinal = std::cos(true_yaw) * dx + std::sin(true_yaw) * dy;
        const double lateral = -std::sin(true_yaw) * dx + std::cos(true_yaw) * dy;
        const double position_error = std::hypot(dx, dy);
        if (position_error >= kSuccessPositionErrorMeters) {
            ROS_WARN_THROTTLE(5.0,
                "[StaticEvaluator] No-offset ICP result exceeded position threshold at grid (%d,%d), yaw %.1f deg: error %.3fm, fitness %.3f",
                point.grid_i, point.grid_j, true_yaw * 180.0 / M_PI, position_error, result.fitness_score);
            continue;
        }

        row.success_count += 1.0;
        sum_fitness += result.fitness_score;
        sum_iterations += iterations;
        if (!std::isnan(latency_ms)) sum_latency += latency_ms;
        sum_long_sq += longitudinal * longitudinal;
        sum_lat_sq += lateral * lateral;
        sum_yaw_sq += dyaw * dyaw;
        sum_cov += result.covariance;
    }

    row.mean_observed_count = row.attempts > 0.0 ? sum_observed_count / row.attempts : kNoData;
    row.success_rate = row.attempts > 0.0 ? row.success_count / row.attempts : 0.0;
    row.has_data = true;
    if (row.success_count > 0.0) {
        row.mean_fitness = sum_fitness / row.success_count;
        row.mean_iterations = sum_iterations / row.success_count;
        row.mean_latency_ms = sum_latency / row.success_count;
        row.longitudinal_rmse = rmse(sum_long_sq, row.success_count);
        row.lateral_rmse = rmse(sum_lat_sq, row.success_count);
        row.yaw_rmse = rmse(sum_yaw_sq, row.success_count);
        
        Eigen::Matrix3d mean_covariance = sum_cov / row.success_count;
        row.cov_xx = mean_covariance(0, 0);
        row.cov_xy = mean_covariance(0, 1);
        row.cov_xyaw = mean_covariance(0, 2);
        row.cov_yx = mean_covariance(1, 0);
        row.cov_yy = mean_covariance(1, 1);
        row.cov_yyaw = mean_covariance(1, 2);
        row.cov_yawx = mean_covariance(2, 0);
        row.cov_yawy = mean_covariance(2, 1);
        row.cov_yawyaw = mean_covariance(2, 2);
        row.cov_trace = mean_covariance.trace();
        row.cov_determinant = mean_covariance.determinant();
    }
    return row;
}


void writeDebugPoseHtmlFiles(const EvaluatorConfig& cfg,
                             const std::vector<LandmarkFeature>& landmarks) {
    if (cfg.debug_poses.empty()) return;

    if (!ensureDirectory(cfg.output_dir)) {
        ROS_WARN("[StaticEvaluator] Failed to create debug output directory: %s", cfg.output_dir.c_str());
        return;
    }

    Eigen::Isometry2d T_rear_bumper = Eigen::Isometry2d::Identity();
    T_rear_bumper.translation() = Eigen::Vector2d(-cfg.rear_axle_offset_x, 0.0);

    IcpParams icp_params;
    icp_params.fitness_threshold = cfg.fitness_threshold;
    icp_params.max_iterations = cfg.max_iterations;
    icp_params.vision_base_std = cfg.vision_base_std;
    icp_params.vision_base_yaw_std = cfg.vision_base_yaw_std;
    icp_params.min_search_radius = cfg.min_search_radius;
    icp_params.max_covariance = cfg.max_covariance;
    icp_params.max_observed_features = cfg.max_observed_features;

    IcpRegistration icp(icp_params);

    std::ofstream index_csv(cfg.output_dir + "/icp_debug_index.csv");
    if (index_csv.is_open()) {
        index_csv << "json_file,html_file,label,x,y,yaw_deg,offset_x,offset_y,offset_yaw_deg\n";
    } else {
        ROS_WARN("[StaticEvaluator] Failed to open ICP debug index CSV in: %s", cfg.output_dir.c_str());
    }

    for (size_t pose_idx = 0; pose_idx < cfg.debug_poses.size(); ++pose_idx) {
        const auto& pose = cfg.debug_poses[pose_idx];
        Eigen::Isometry2d T_map_rear_true = Eigen::Isometry2d::Identity();
        T_map_rear_true.translation() = Eigen::Vector2d(pose.x, pose.y);
        T_map_rear_true.linear() = Eigen::Rotation2Dd(pose.yaw).toRotationMatrix();
        const Eigen::Isometry2d T_map_bumper_true = T_map_rear_true * T_rear_bumper;

        const std::vector<LocalFeature> observed = createVisibleFeatures(landmarks, T_map_bumper_true, cfg.sensor_range);
        // Introduce configured initial guess offset to visualize convergence
        Eigen::Isometry2d T_offset = Eigen::Isometry2d::Identity();
        T_offset.translation() = Eigen::Vector2d(pose.offset_x, pose.offset_y);
        T_offset.linear() = Eigen::Rotation2Dd(pose.offset_yaw).toRotationMatrix();
        const Eigen::Isometry2d T_initial_guess = T_map_bumper_true * T_offset;
        RegistrationResult result = icp.align(observed, landmarks, T_initial_guess, true, false, true);

        std::vector<LocalFeature> moving_landmarks;
        moving_landmarks.reserve(landmarks.size());
        for (const auto& landmark : landmarks) {
            LocalFeature feature;
            feature.x = landmark.x;
            feature.y = landmark.y;
            feature.cov_xx = 0.0;
            feature.cov_yy = 0.0;
            feature.cov_xy = 0.0;
            feature.class_id = landmark.class_id;
            moving_landmarks.push_back(feature);
        }
        RegistrationResult landmark_to_landmark_result = icp.align(
            moving_landmarks, landmarks, T_map_rear_true, true, false, true);

        const std::string filename = debugPoseFileName(pose_idx);
        const std::string html_filename = filename.substr(0, filename.size() - 5) + ".html";
        const std::string json_path = cfg.output_dir + "/" + filename;
        if (index_csv.is_open()) {
            index_csv << filename << "," << html_filename << "," << pose.label << ","
                      << pose.x << "," << pose.y << "," << pose.yaw * 180.0 / M_PI << ","
                      << pose.offset_x << "," << pose.offset_y << ","
                      << pose.offset_yaw * 180.0 / M_PI << "\n";
        }
        if (writeDebugJson(json_path, pose, cfg.rear_axle_offset_x, cfg.sensor_range, landmarks, observed, result,
                           moving_landmarks, landmark_to_landmark_result)) {
            ROS_INFO("[StaticEvaluator] ICP debug JSON written to: %s", json_path.c_str());
        } else {
            ROS_WARN("[StaticEvaluator] Failed to write ICP debug JSON: %s", json_path.c_str());
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_evaluator");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    EvaluatorConfig cfg;
    if (!loadConfig(nh, pnh, cfg)) {
        ROS_ERROR("[StaticEvaluator] Parameter 'landmark_file' is required");
        return 1;
    }

    OsmLandmarkLoader landmark_loader(0.05);
    if (!landmark_loader.load(cfg.landmark_file)) {
        ROS_ERROR("[StaticEvaluator] Failed to load landmarks from %s", cfg.landmark_file.c_str());
        return 1;
    }

    const nav_msgs::OccupancyGrid grid = landmark_loader.getOccupancyGrid();
    const std::vector<GridCell> points = generateGridPoints(grid, cfg);
    const std::vector<LandmarkFeature> landmarks = landmark_loader.queryNear(0.0, 0.0, -1.0);
    if (points.empty() || landmarks.empty()) {
        ROS_ERROR("[StaticEvaluator] No grid points or landmarks available");
        return 1;
    }

    if (writeLandmarksCsv(cfg.landmarks_csv_path, landmarks)) {
        ROS_INFO("[StaticEvaluator] Landmarks CSV written to: %s", cfg.landmarks_csv_path.c_str());
    } else {
        ROS_WARN("[StaticEvaluator] Failed to write landmarks CSV: %s", cfg.landmarks_csv_path.c_str());
    }

    writeDebugPoseHtmlFiles(cfg, landmarks);

    std::vector<EvaluationRow> rows(points.size());
    int completed = 0;
    ROS_INFO("[StaticEvaluator] Evaluating %zu grid cells with %.2fm step", points.size(), cfg.grid_step);

    #pragma omp parallel for schedule(static, 1)
    for (size_t idx = 0; idx < points.size(); ++idx) {
        rows[idx] = evaluatePoint(points[idx], cfg, landmarks);
        int local_completed = 0;
        #pragma omp atomic capture
        local_completed = ++completed;
        if (local_completed % 10 == 0 || local_completed == static_cast<int>(points.size())) {
            ROS_INFO("[StaticEvaluator] Evaluated %d/%zu cells", local_completed, points.size());
        }
    }

    std::ofstream csv(cfg.output_csv_path);
    if (!csv.is_open()) {
        ROS_ERROR("[StaticEvaluator] Failed to open output CSV: %s", cfg.output_csv_path.c_str());
        return 1;
    }
    writeCsvHeader(csv);
    for (const auto& row : rows) {
        if (row.has_data) writeCsvRow(csv, row);
    }
    csv.close();

    ROS_INFO("[StaticEvaluator] CSV written to: %s", cfg.output_csv_path.c_str());
    return 0;
}
