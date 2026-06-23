#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <carmaker_msgs/DynamicsInfo.h>
#include <carmaker_msgs/IcpRegistrationMetrics.h>
#include "carmaker_localization/evaluation_utils.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <unistd.h>
#include <vector>

using namespace carmaker_localization;

namespace {

constexpr const char* kTopicGt = "/carmaker/dynamic_info";
constexpr const char* kTopicIcpMetrics = "/localization/debug/icp_metrics";

std::vector<std::string> collectBagFiles(const std::string& path) {
    std::vector<std::string> bags;
    struct stat s;
    if (stat(path.c_str(), &s) == 0) {
        if (s.st_mode & S_IFDIR) {
            DIR* dir = opendir(path.c_str());
            if (dir) {
                struct dirent* entry;
                while ((entry = readdir(dir)) != nullptr) {
                    std::string name = entry->d_name;
                    if (name.size() > 4 && name.substr(name.size() - 4) == ".bag") {
                        // Append slash if not present
                        std::string full_path = path;
                        if (full_path.back() != '/') full_path += "/";
                        bags.push_back(full_path + name);
                    }
                }
                closedir(dir);
            }
        } else if (s.st_mode & S_IFREG) {
            bags.push_back(path);
        }
    }
    std::sort(bags.begin(), bags.end());
    return bags;
}

struct Config {
    std::string bag_path;
    std::string output_csv_path = "grid_registration_results.csv";
    double center_x = 0.0;
    double center_y = -4.33;
    double grid_step = 1.0;
};

struct GtSample {
    double time = 0.0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct MetricSample {
    double time = 0.0;
    carmaker_msgs::IcpRegistrationMetrics msg;
};

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

template <typename MsgT>
double messageTime(const MsgT& msg, const ros::Time& bag_time) {
    return msg.header.stamp.toSec() > 0.0 ? msg.header.stamp.toSec() : bag_time.toSec();
}

GridCell gridFromXY(double x, double y, const Config& cfg) {
    GridCell cell;
    cell.grid_i = static_cast<int>(std::llround((x - cfg.center_x) / cfg.grid_step));
    cell.grid_j = static_cast<int>(std::llround((y - cfg.center_y) / cfg.grid_step));
    cell.cell_x = cfg.center_x + cell.grid_i * cfg.grid_step;
    cell.cell_y = cfg.center_y + cell.grid_j * cfg.grid_step;
    return cell;
}

template <typename SampleT>
const SampleT* nearestSample(const std::vector<SampleT>& samples, double query_time) {
    if (samples.empty()) return nullptr;

    auto it = std::lower_bound(samples.begin(), samples.end(), query_time,
                               [](const SampleT& sample, double time) { return sample.time < time; });
    const SampleT* best = nullptr;
    if (it != samples.end()) best = &(*it);
    if (it != samples.begin()) {
        const SampleT* prev = &(*std::prev(it));
        if (!best || std::abs(prev->time - query_time) < std::abs(best->time - query_time)) {
            best = prev;
        }
    }
    return best;
}

double weightedMeanOrNoData(double weighted_sum, double weight) {
    return weight > 0.0 ? weighted_sum / weight : kNoData;
}

double weightedRmseOrNoData(double weighted_sum_sq, double weight) {
    return weight > 0.0 ? std::sqrt(weighted_sum_sq / weight) : kNoData;
}

bool loadConfig(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, Config& cfg) {
    // 1. 글로벌/공통 파라미터 조회 (낮은 우선순위)
    nh.param("evaluator/bag/path", cfg.bag_path, cfg.bag_path);

    // 2. 노드 프라이빗 파라미터 조회 (높은 우선순위)
    pnh.param("evaluator/bag/path", cfg.bag_path, cfg.bag_path);
    pnh.param("bag/path", cfg.bag_path, cfg.bag_path);
    pnh.param("bag_path", cfg.bag_path, cfg.bag_path);
    if (cfg.bag_path.empty()) return false;

    pnh.param("center_x", cfg.center_x, cfg.center_x);
    pnh.param("center_y", cfg.center_y, cfg.center_y);
    pnh.param("grid_step", cfg.grid_step, cfg.grid_step);
    pnh.param("output_csv_path", cfg.output_csv_path, cfg.output_csv_path);

    nh.param("evaluator/center_x", cfg.center_x, cfg.center_x);
    nh.param("evaluator/center_y", cfg.center_y, cfg.center_y);
    nh.param("evaluator/grid_step", cfg.grid_step, cfg.grid_step);
    nh.param("evaluator/output_csv_path", cfg.output_csv_path, cfg.output_csv_path);

    cfg.grid_step = std::max(1e-3, cfg.grid_step);
    cfg.output_csv_path = resolvePackagePath(cfg.output_csv_path);
    return true;
}

bool readBagSamples(const std::string& bag_path,
                    std::vector<GtSample>& gt_samples,
                    std::vector<MetricSample>& metric_samples) {
    rosbag::Bag bag;
    try {
        bag.open(bag_path, rosbag::bagmode::Read);
        std::vector<std::string> topics = {kTopicGt, kTopicIcpMetrics};
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        // Preallocate memory space for vectors based on bag view size to optimize insertion performance
        const size_t estimated_size = view.size() / 2;
        gt_samples.reserve(estimated_size);
        metric_samples.reserve(estimated_size);

        for (const rosbag::MessageInstance& item : view) {
            const std::string& topic = item.getTopic();
            if (topic == kTopicGt || topic == std::string("/") + kTopicGt) {
                const auto msg = item.instantiate<carmaker_msgs::DynamicsInfo>();
                if (msg && std::isfinite(msg->RearAxle_x) && std::isfinite(msg->RearAxle_y)) {
                    gt_samples.push_back({messageTime(*msg, item.getTime()), msg->RearAxle_x, msg->RearAxle_y, msg->Car_Yaw});
                }
            } else if (topic == kTopicIcpMetrics || topic == std::string("/") + kTopicIcpMetrics) {
                const auto msg = item.instantiate<carmaker_msgs::IcpRegistrationMetrics>();
                if (msg) metric_samples.push_back({messageTime(*msg, item.getTime()), *msg});
            }
        }
        bag.close();
    } catch (const std::exception& e) {
        ROS_ERROR("[BagEvaluator] Failed to read bag %s: %s", bag_path.c_str(), e.what());
        return false;
    }

    auto by_time = [](const auto& a, const auto& b) { return a.time < b.time; };
    std::sort(gt_samples.begin(), gt_samples.end(), by_time);
    std::sort(metric_samples.begin(), metric_samples.end(), by_time);
    return true;
}

struct RowAccumulator {
    EvaluationRow row;
    double observed_weighted_sum = 0.0;
    double landmarks_weighted_sum = 0.0;
    double fitness_weighted_sum = 0.0;
    double iterations_weighted_sum = 0.0;
    double latency_weighted_sum = 0.0;
    double long_rmse_sum_sq = 0.0;
    double lat_rmse_sum_sq = 0.0;
    double yaw_rmse_sum_sq = 0.0;
    double cov_xx_weighted_sum = 0.0;
    double cov_xy_weighted_sum = 0.0;
    double cov_xyaw_weighted_sum = 0.0;
    double cov_yx_weighted_sum = 0.0;
    double cov_yy_weighted_sum = 0.0;
    double cov_yyaw_weighted_sum = 0.0;
    double cov_yawx_weighted_sum = 0.0;
    double cov_yawy_weighted_sum = 0.0;
    double cov_yawyaw_weighted_sum = 0.0;
    double cov_trace_weighted_sum = 0.0;
    double cov_det_weighted_sum = 0.0;
    double observed_weight = 0.0;
    double landmarks_weight = 0.0;
    double cov_xx_weight = 0.0;
    double cov_xy_weight = 0.0;
    double cov_xyaw_weight = 0.0;
    double cov_yx_weight = 0.0;
    double cov_yy_weight = 0.0;
    double cov_yyaw_weight = 0.0;
    double cov_yawx_weight = 0.0;
    double cov_yawy_weight = 0.0;
    double cov_yawyaw_weight = 0.0;
    double cov_trace_weight = 0.0;
    double cov_det_weight = 0.0;
};

void addFinite(double value, double& weighted_sum, double& weight_sum) {
    if (!std::isfinite(value)) return;
    weighted_sum += value;
    weight_sum += 1.0;
}

void addMetricSample(RowAccumulator& acc, const Config& cfg, const GtSample& gt,
                     const MetricSample& metric) {
    if (acc.row.attempts == 0.0 && acc.row.success_count == 0.0) {
        acc.row.grid = gridFromXY(gt.x, gt.y, cfg);
    }

    acc.row.attempts += 1.0;
    addFinite(static_cast<double>(metric.msg.observed_count), acc.observed_weighted_sum, acc.observed_weight);
    addFinite(static_cast<double>(metric.msg.landmark_count), acc.landmarks_weighted_sum, acc.landmarks_weight);

    if (!metric.msg.success ||
        !std::isfinite(metric.msg.icp_rear_x) ||
        !std::isfinite(metric.msg.icp_rear_y) ||
        !std::isfinite(metric.msg.icp_rear_yaw)) {
        return;
    }

    const double dx = metric.msg.icp_rear_x - gt.x;
    const double dy = metric.msg.icp_rear_y - gt.y;
    const double longitudinal = std::cos(gt.yaw) * dx + std::sin(gt.yaw) * dy;
    const double lateral = -std::sin(gt.yaw) * dx + std::cos(gt.yaw) * dy;
    const double yaw_error = normalizeAngle(metric.msg.icp_rear_yaw - gt.yaw);

    acc.row.success_count += 1.0;
    acc.fitness_weighted_sum += metric.msg.fitness_score;
    acc.iterations_weighted_sum += static_cast<double>(metric.msg.iterations);
    acc.latency_weighted_sum += metric.msg.latency_ms;
    acc.long_rmse_sum_sq += longitudinal * longitudinal;
    acc.lat_rmse_sum_sq += lateral * lateral;
    acc.yaw_rmse_sum_sq += yaw_error * yaw_error;

    addFinite(metric.msg.cov_xx, acc.cov_xx_weighted_sum, acc.cov_xx_weight);
    addFinite(metric.msg.cov_xy, acc.cov_xy_weighted_sum, acc.cov_xy_weight);
    addFinite(metric.msg.cov_xyaw, acc.cov_xyaw_weighted_sum, acc.cov_xyaw_weight);
    addFinite(metric.msg.cov_yx, acc.cov_yx_weighted_sum, acc.cov_yx_weight);
    addFinite(metric.msg.cov_yy, acc.cov_yy_weighted_sum, acc.cov_yy_weight);
    addFinite(metric.msg.cov_yyaw, acc.cov_yyaw_weighted_sum, acc.cov_yyaw_weight);
    addFinite(metric.msg.cov_yawx, acc.cov_yawx_weighted_sum, acc.cov_yawx_weight);
    addFinite(metric.msg.cov_yawy, acc.cov_yawy_weighted_sum, acc.cov_yawy_weight);
    addFinite(metric.msg.cov_yawyaw, acc.cov_yawyaw_weighted_sum, acc.cov_yawyaw_weight);
    addFinite(metric.msg.cov_trace, acc.cov_trace_weighted_sum, acc.cov_trace_weight);
    addFinite(metric.msg.cov_determinant, acc.cov_det_weighted_sum, acc.cov_det_weight);
}

std::vector<EvaluationRow> finalizeRows(std::map<std::pair<int, int>, RowAccumulator>& accumulators) {
    std::vector<EvaluationRow> aggregated;
    aggregated.reserve(accumulators.size());
    for (auto& item : accumulators) {
        auto& acc = item.second;
        const double attempts = acc.row.attempts;
        const double successes = acc.row.success_count;
        acc.row.success_rate = attempts > 0.0 ? successes / attempts : 0.0;
        acc.row.mean_observed_count = weightedMeanOrNoData(acc.observed_weighted_sum, acc.observed_weight);
        acc.row.landmark_count = weightedMeanOrNoData(acc.landmarks_weighted_sum, acc.landmarks_weight);
        acc.row.mean_fitness = weightedMeanOrNoData(acc.fitness_weighted_sum, successes);
        acc.row.mean_iterations = weightedMeanOrNoData(acc.iterations_weighted_sum, successes);
        acc.row.mean_latency_ms = weightedMeanOrNoData(acc.latency_weighted_sum, successes);
        acc.row.longitudinal_rmse = weightedRmseOrNoData(acc.long_rmse_sum_sq, successes);
        acc.row.lateral_rmse = weightedRmseOrNoData(acc.lat_rmse_sum_sq, successes);
        acc.row.yaw_rmse = weightedRmseOrNoData(acc.yaw_rmse_sum_sq, successes);
        acc.row.cov_xx = weightedMeanOrNoData(acc.cov_xx_weighted_sum, acc.cov_xx_weight);
        acc.row.cov_xy = weightedMeanOrNoData(acc.cov_xy_weighted_sum, acc.cov_xy_weight);
        acc.row.cov_xyaw = weightedMeanOrNoData(acc.cov_xyaw_weighted_sum, acc.cov_xyaw_weight);
        acc.row.cov_yx = weightedMeanOrNoData(acc.cov_yx_weighted_sum, acc.cov_yx_weight);
        acc.row.cov_yy = weightedMeanOrNoData(acc.cov_yy_weighted_sum, acc.cov_yy_weight);
        acc.row.cov_yyaw = weightedMeanOrNoData(acc.cov_yyaw_weighted_sum, acc.cov_yyaw_weight);
        acc.row.cov_yawx = weightedMeanOrNoData(acc.cov_yawx_weighted_sum, acc.cov_yawx_weight);
        acc.row.cov_yawy = weightedMeanOrNoData(acc.cov_yawy_weighted_sum, acc.cov_yawy_weight);
        acc.row.cov_yawyaw = weightedMeanOrNoData(acc.cov_yawyaw_weighted_sum, acc.cov_yawyaw_weight);
        acc.row.cov_trace = weightedMeanOrNoData(acc.cov_trace_weighted_sum, acc.cov_trace_weight);
        acc.row.cov_determinant = weightedMeanOrNoData(acc.cov_det_weighted_sum, acc.cov_det_weight);
        acc.row.has_data = attempts > 0.0;
        aggregated.push_back(acc.row);
    }
    return aggregated;
}

void accumulateBag(const Config& cfg,
                   const std::vector<GtSample>& gt_samples,
                   const std::vector<MetricSample>& metric_samples,
                   std::map<std::pair<int, int>, RowAccumulator>& accumulators) {
    for (const auto& metric : metric_samples) {
        const GtSample* gt = nearestSample(gt_samples, metric.time);
        if (!gt) continue;

        const GridCell grid = gridFromXY(gt->x, gt->y, cfg);
        auto& acc = accumulators[std::make_pair(grid.grid_i, grid.grid_j)];
        addMetricSample(acc, cfg, *gt, metric);
    }
}

bool writeCsv(const std::string& path, const std::vector<EvaluationRow>& rows) {
    std::ofstream csv(path);
    if (!csv.is_open()) return false;
    writeCsvHeader(csv);
    for (const auto& row : rows) {
        writeCsvRow(csv, row);
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_evaluator");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    Config cfg;
    if (!loadConfig(nh, pnh, cfg)) {
        ROS_ERROR("[BagEvaluator] Parameter 'bag_path' is required");
        return 1;
    }

    std::vector<std::string> bag_files = collectBagFiles(cfg.bag_path);
    if (bag_files.empty()) {
        ROS_ERROR("[BagEvaluator] No bag files found at path: %s", cfg.bag_path.c_str());
        return 1;
    }

    std::map<std::pair<int, int>, RowAccumulator> accumulators;
    size_t evaluated_bag_count = 0;
    for (const auto& bag_file : bag_files) {
        std::vector<GtSample> gt_samples;
        std::vector<MetricSample> metric_samples;
        ROS_INFO("[BagEvaluator] Reading bag: %s", bag_file.c_str());
        if (!readBagSamples(bag_file, gt_samples, metric_samples)) {
            ROS_WARN("[BagEvaluator] Skipping corrupted or failed bag: %s", bag_file.c_str());
            continue;
        }
        if (gt_samples.empty() || metric_samples.empty()) {
            ROS_WARN("[BagEvaluator] Missing GT or ICP metrics topics in bag %s, skipping", bag_file.c_str());
            continue;
        }
        ROS_INFO("[BagEvaluator] Evaluating bag: %s", bag_file.c_str());
        accumulateBag(cfg, gt_samples, metric_samples, accumulators);
        ++evaluated_bag_count;
    }

    if (accumulators.empty()) {
        ROS_ERROR("[BagEvaluator] No valid bag files were successfully evaluated");
        return 1;
    }

    const std::vector<EvaluationRow> aggregated_rows = finalizeRows(accumulators);
    if (!writeCsv(cfg.output_csv_path, aggregated_rows)) {
        ROS_ERROR("[BagEvaluator] Failed to open output CSV: %s", cfg.output_csv_path.c_str());
        return 1;
    }

    ROS_INFO("[BagEvaluator] Successfully wrote %zu grid row(s) from %zu bag file(s) to CSV: %s",
             aggregated_rows.size(), evaluated_bag_count, cfg.output_csv_path.c_str());
    return 0;
}
