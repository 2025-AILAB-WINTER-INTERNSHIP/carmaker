/**
 * @file types.h
 * @brief Consolidated data structures for carmaker_planning.
 */
#ifndef CARMAKER_PLANNING_TYPES_H
#define CARMAKER_PLANNING_TYPES_H

#include <cstdint>
#include <vector>
#include <cmath>
#include <string>
#include <limits>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "carmaker_planning/math.h"

namespace carmaker_planning {

struct State {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double v = 0.0;
  double a = 0.0;
  double theta_rate = 0.0;

  State() = default;
  State(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}
  void normalize() { theta = wrap_to_pi(theta); }
};

struct PathPoint {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double v = 0.0;
  double a = 0.0;
  double s = 0.0;
  int direction = 1;  // 1: forward, -1: reverse
  double t = 0.0;
  double d_lat = 0.0;

  PathPoint() = default;
  PathPoint(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}
};

using Path = std::vector<PathPoint>;

struct CollisionCircle {
  double offset;
  double radius;
};

struct VehicleSpec {
  double width;
  double length;
  double wheelbase;
  double rear_axle_offset;
  double min_turning_radius;
  double max_steer_angle;
  double max_vel;
  double max_accel;
  double max_decel;
  double max_jerk;
  double max_lat_acc;
  std::vector<CollisionCircle> collision_circles;
};

enum class DubinsPathType { LSL=0, LSR, RSL, RSR, RLR, LRL };

struct DubinsPath {
  DubinsPathType type;
  double t, p, q, total_length, rho;
  std::vector<PathPoint> points;
};

enum class RSPathType {
  LfSfLf, LfSfRf, RfSfLf, RfSfRf,
  LfRbLf, RfLbRf,
  LfRbLb, RfLbRb,
  LbSbRb, RbSbLb, LbSbLb, RbSbRb,
  UNKNOWN
};

struct RSPath {
  RSPathType type = RSPathType::UNKNOWN;
  double total_length = 0.0;
  double rho = 1.0;
  std::vector<double> seg_lengths; // normalized lengths (t, u, v, etc.)
  std::vector<int> seg_dir;        // 1: forward, -1: reverse
  std::vector<int> seg_steer;      // 1: left, 0: straight, -1: right
  std::vector<PathPoint> points;
};

enum class PlanningStatus {
  INVALID = 0,
  SUCCESS_OPTIMAL,
  SUCCESS_BEST_EFFORT,
  FAILURE_NO_PATH,
  FAILURE_TIMEOUT,
  FAILURE_COLLISION,
  FAILURE_COLLISION_START,
  FAILURE_COLLISION_GOAL,
  FAILURE_POST_PROCESSING,
  FAILURE_INVALID_MAP
};

inline const char* statusString(PlanningStatus s) {
  switch (s) {
    case PlanningStatus::SUCCESS_OPTIMAL:        return "SUCCESS_OPTIMAL";
    case PlanningStatus::SUCCESS_BEST_EFFORT:    return "SUCCESS_BEST_EFFORT";
    case PlanningStatus::FAILURE_NO_PATH:        return "FAILURE_NO_PATH";
    case PlanningStatus::FAILURE_TIMEOUT:        return "FAILURE_TIMEOUT";
    case PlanningStatus::FAILURE_COLLISION:      return "FAILURE_COLLISION";
    case PlanningStatus::FAILURE_COLLISION_START: return "FAILURE_COLLISION_START";
    case PlanningStatus::FAILURE_COLLISION_GOAL:  return "FAILURE_COLLISION_GOAL";
    case PlanningStatus::FAILURE_POST_PROCESSING: return "FAILURE_POST_PROCESSING";
    case PlanningStatus::FAILURE_INVALID_MAP:    return "FAILURE_INVALID_MAP";
    default: return "UNKNOWN";
  }
}

struct TrajectoryDiagnostic {
  bool is_valid = false;
  bool curvature_ok = false;
  bool yaw_ok = false;
  bool time_ok = false;
  bool vel_ok = false;
  bool acc_ok = false;
  bool jerk_ok = false;
  bool steer_vel_ok = false;

  int curv_violations = -1;
  int yaw_violations = -1;
  int time_violations = -1;
  int vel_violations = -1;
  int acc_violations = -1;
  int jerk_violations = -1;
  int steer_vel_violations = -1;

  double max_curv_violation = -1.0;
  double max_yaw_error_rad = -1.0;
  double max_time_error_sec = -1.0;
  double max_vel_violation = -1.0;
  double max_acc_violation = -1.0;
  double max_jerk_violation = -1.0;
  double max_steer_vel_violation = -1.0;
};

struct GlobalPlanningResult {
  PlanningStatus status = PlanningStatus::INVALID;
  Path path;
  Path raw_path;
  double planning_time = -1.0, smoothing_time = -1.0;
  double resampling_time = -1.0, profiling_time = -1.0, total_time = -1.0;
  int expanded_nodes = -1, search_iterations = -1;
  std::vector<std::pair<std::string, std::string>> logs;
  TrajectoryDiagnostic diagnostic;

  void addLog(const std::string& level, const std::string& msg) {
    logs.push_back({level, msg});
  }
  void warn(const std::string& msg) { addLog("WARN", msg); }
  void info(const std::string& msg) { addLog("INFO", msg); }
  void error(const std::string& msg) { addLog("ERROR", msg); }
  void debug(const std::string& msg) { addLog("DEBUG", msg); }

  bool success() const {
    return status == PlanningStatus::SUCCESS_OPTIMAL ||
           status == PlanningStatus::SUCCESS_BEST_EFFORT;
  }
  const char* statusString() const { return carmaker_planning::statusString(status); }
};

struct GlobalPlanningDiagnostic {
  PlanningStatus status = PlanningStatus::INVALID;
  double total_time = -1.0;
  double planning_time = -1.0;
  double smoothing_time = -1.0;
  double resampling_time = -1.0;
  double profiling_time = -1.0;
  int expanded_nodes = -1;
  int search_iterations = -1;
  double path_length = -1.0;
  TrajectoryDiagnostic diagnostic;

  GlobalPlanningDiagnostic() = default;

  // Lightweight conversion constructor to avoid deep-copying heavy path vectors
  explicit GlobalPlanningDiagnostic(const GlobalPlanningResult& res)
    : status(res.status),
      total_time(res.total_time),
      planning_time(res.planning_time),
      smoothing_time(res.smoothing_time),
      resampling_time(res.resampling_time),
      profiling_time(res.profiling_time),
      expanded_nodes(res.expanded_nodes),
      search_iterations(res.search_iterations),
      path_length(res.path.empty() ? -1.0 : res.path.back().s),
      diagnostic(res.diagnostic) {}

  const char* statusString() const { return carmaker_planning::statusString(status); }
};

struct GlobalGridIndex { int x=0, y=0, theta=0; };

struct GlobalNode3D {
  GlobalGridIndex idx;
  double x=0, y=0, theta=0;
  double g_cost=0, h_cost=0;
  const GlobalNode3D* parent = nullptr;
  double steering = 0.0;
  int direction = 1;
  bool is_closed = false;
  double getF() const { return g_cost + h_cost; }
  struct Greater {
    bool operator()(const GlobalNode3D* a, const GlobalNode3D* b) const
    { return a->getF() > b->getF(); }
  };
};

struct GlobalNode2D {
  int x_idx, y_idx; double cost;
  struct Greater {
    bool operator()(const GlobalNode2D& a, const GlobalNode2D& b) const
    { return a.cost > b.cost; }
  };
};

struct GridMapConfig {
  int lethal_threshold;
  bool unknown_cost_free;
  uint8_t unknown_value;
  uint8_t occupancy_value;
};

struct GoalTolerance {
  double xy = 0.2;
  double theta = 0.2;
};

struct GlobalCostmapConfig {
  GridMapConfig grid_map;
};

struct GlobalPlannerConfig {
  int max_iterations;
  double max_planning_time;
  double xy_resolution, theta_resolution;
  GoalTolerance goal_tolerance;
  int heuristic_scale;
  double step_size;
  int next_node_num, analytic_expansion_ratio;
  struct Weights { double turn, reverse, change_dir, heuristic; } weights;
};

struct GlobalPostProcessConfig {
  struct Smoother {
    double weight_data, weight_smooth, tolerance;
    int max_iterations;
  } smoother;
  struct Resampler {
    double resolution;
    int yaw_blending_width;
    bool use_spline;
    double cusp_angular_threshold_rad;
  } resampler;
  struct Profiler {
    double max_vel, max_accel, max_decel, max_jerk, max_lat_acc, goal_vel;
    double gear_shift_duration;
    double min_velocity_denominator;
  } profiler;
  struct Visualization {
    double arrow_spacing_meters;
  } visualization;
};

struct GlobalMainConfig {
  VehicleSpec vehicle;
  GlobalCostmapConfig global_costmap;
  GlobalPlannerConfig planner;
  GlobalPostProcessConfig post_process;
};

// ── Quaternion Helpers ────────────────────────────────────────────────
inline geometry_msgs::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::Quaternion q;
  q.x = 0.0; q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

inline double quaternionToYaw(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline void loadVehicleSpec(const ros::NodeHandle& nh, VehicleSpec& spec,
                            const std::string& ns = "vehicle") {
  nh.param(ns + "/width",             spec.width,            1.9);
  nh.param(ns + "/length",            spec.length,           4.635);
  nh.param(ns + "/wheelbase",         spec.wheelbase,        3.0);

  if (!nh.getParam(ns + "/rear_axle_offset", spec.rear_axle_offset)) {
    nh.param(ns + "/rear_axle_offset_x", spec.rear_axle_offset, 0.79);
  }

  nh.param(ns + "/min_turning_radius", spec.min_turning_radius, 5.2);

  double steer_deg = 30.0;
  nh.param(ns + "/max_steer_angle_deg", steer_deg, steer_deg);
  spec.max_steer_angle = deg2rad(steer_deg);

  nh.param(ns + "/limits/max_vel",   spec.max_vel,   1.5);
  nh.param(ns + "/limits/max_accel", spec.max_accel, 1.0);
  nh.param(ns + "/limits/max_decel", spec.max_decel, 2.0);
  nh.param(ns + "/limits/max_jerk",  spec.max_jerk,  2.0);
  nh.param(ns + "/limits/max_lat_acc", spec.max_lat_acc, 1.5);

  std::vector<double> offsets, radii;
  nh.getParam(ns + "/collision_circles/offsets", offsets);
  nh.getParam(ns + "/collision_circles/radii",   radii);

  spec.collision_circles.clear();
  if (offsets.size() == radii.size()) {
    for (size_t i = 0; i < offsets.size(); ++i) {
      spec.collision_circles.push_back({offsets[i], radii[i]});
    }
  }
}

inline void loadGlobalCostmapConfig(const ros::NodeHandle& nh, GlobalCostmapConfig& cfg,
                                    const std::string& ns = "global_costmap") {
  nh.param(ns + "/grid_map/lethal_threshold",  cfg.grid_map.lethal_threshold,  50);
  nh.param(ns + "/grid_map/unknown_cost_free", cfg.grid_map.unknown_cost_free, true);
  int uv = 255;
  nh.param(ns + "/grid_map/unknown_value", uv, uv);
  cfg.grid_map.unknown_value = static_cast<uint8_t>(uv);
  int ov = 100;
  nh.param(ns + "/grid_map/occupancy_value", ov, ov);
  cfg.grid_map.occupancy_value = static_cast<uint8_t>(ov);
}

inline void loadGlobalPlannerConfig(const ros::NodeHandle& nh, GlobalPlannerConfig& cfg,
                                    const std::string& ns = "global_planner") {
  nh.param(ns + "/max_iterations",          cfg.max_iterations,         10000);
  nh.param(ns + "/max_planning_time",       cfg.max_planning_time,      2.0);
  nh.param(ns + "/xy_resolution",           cfg.xy_resolution,          0.05);

  double theta_deg = 5.0;
  nh.param(ns + "/theta_resolution_deg",   theta_deg, theta_deg);
  cfg.theta_resolution = deg2rad(theta_deg);

  nh.param(ns + "/step_size",               cfg.step_size,              0.5);
  nh.param(ns + "/next_node_num",           cfg.next_node_num,          7);
  nh.param(ns + "/goal_tolerance_xy",       cfg.goal_tolerance.xy,      0.2);

  double goal_theta_deg = 11.5;
  nh.param(ns + "/goal_tolerance_theta_deg", goal_theta_deg, goal_theta_deg);
  cfg.goal_tolerance.theta = deg2rad(goal_theta_deg);

  nh.param(ns + "/analytic_expansion_ratio", cfg.analytic_expansion_ratio, 5);
  nh.param(ns + "/heuristic_scale",         cfg.heuristic_scale,        2);
  nh.param(ns + "/weights/turn",            cfg.weights.turn,           2.0);
  nh.param(ns + "/weights/reverse",         cfg.weights.reverse,        5.0);
  nh.param(ns + "/weights/change_dir",      cfg.weights.change_dir,     15.0);
  nh.param(ns + "/weights/heuristic",       cfg.weights.heuristic,      1.0);
}

inline void loadGlobalPostProcessConfig(const ros::NodeHandle& nh, GlobalPostProcessConfig& cfg,
                                        const std::string& ns = "global_post_processing") {
  nh.param(ns + "/smoother/weight_data",    cfg.smoother.weight_data,   0.2);
  nh.param(ns + "/smoother/weight_smooth",  cfg.smoother.weight_smooth, 0.35);
  nh.param(ns + "/smoother/tolerance",      cfg.smoother.tolerance,     0.001);
  nh.param(ns + "/smoother/max_iterations", cfg.smoother.max_iterations, 500);
  nh.param(ns + "/resampler/resolution",    cfg.resampler.resolution,   0.1);
  nh.param(ns + "/resampler/yaw_blending_width", cfg.resampler.yaw_blending_width, 5);
  nh.param(ns + "/resampler/use_spline",    cfg.resampler.use_spline,   true);

  double cusp_threshold_deg = 20.0;
  nh.param(ns + "/resampler/cusp_angular_threshold_deg", cusp_threshold_deg, cusp_threshold_deg);
  cfg.resampler.cusp_angular_threshold_rad = deg2rad(cusp_threshold_deg);

  nh.param(ns + "/profiler/max_vel",        cfg.profiler.max_vel,       1.5);
  nh.param(ns + "/profiler/max_accel",      cfg.profiler.max_accel,     1.0);
  nh.param(ns + "/profiler/max_decel",      cfg.profiler.max_decel,     1.5);
  nh.param(ns + "/profiler/max_jerk",       cfg.profiler.max_jerk,      5.0);
  nh.param(ns + "/profiler/max_lat_acc",    cfg.profiler.max_lat_acc,   1.0);
  nh.param(ns + "/profiler/goal_vel",       cfg.profiler.goal_vel,      0.0);
  nh.param(ns + "/profiler/gear_shift_duration", cfg.profiler.gear_shift_duration, 1.2);
  nh.param(ns + "/profiler/min_velocity_denominator", cfg.profiler.min_velocity_denominator, 0.02);

  nh.param(ns + "/visualization/arrow_spacing_meters", cfg.visualization.arrow_spacing_meters, 0.2);
}

inline void loadGlobalMainConfig(const ros::NodeHandle& nh, GlobalMainConfig& cfg) {
  loadVehicleSpec(nh, cfg.vehicle);
  loadGlobalCostmapConfig(nh, cfg.global_costmap);
  loadGlobalPlannerConfig(nh, cfg.planner);
  loadGlobalPostProcessConfig(nh, cfg.post_process);
}

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_TYPES_H
