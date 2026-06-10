/**
 * @file post_processing.cpp
 * @brief Consolidated implementation for post-processing pipeline steps.
 */

#include "carmaker_planning/post_processing.h"
#include "carmaker_planning/math.h"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>

namespace carmaker_planning {

namespace {
/**
 * @brief Template helper to validate if path points exceed a specific dynamic limit,
 * updating diagnostic flags and violation counts.
 */
template <typename ValFunc>
void checkLimit(const Path& path, bool& ok_flag, int& violations, double& max_violation,
                double limit, double epsilon, ValFunc val_func, size_t start_idx = 0) {
  for (size_t i = start_idx; i < path.size(); ++i) {
    double val = val_func(i);
    if (val > limit + epsilon) {
      ok_flag = false;
      violations++;
      max_violation = std::max(max_violation, val - limit);
    }
  }
}

/**
 * @brief Apply 1D Moving Average filter to a sequence of double values.
 */
void applyMovingAverage(std::vector<double>& data, int half_window, int passes) {
  const int n = static_cast<int>(data.size());
  if (n <= 2 * half_window) return;
  for (int iter = 0; iter < passes; ++iter) {
    std::vector<double> temp = data;
    for (int i = 0; i < n; ++i) {
      double sum = 0.0;
      int count = 0;
      for (int w = -half_window; w <= half_window; ++w) {
        int idx = i + w;
        if (idx >= 0 && idx < n) {
          sum += temp[idx];
          count++;
        }
      }
      data[i] = sum / count;
    }
  }
}
} // namespace


PostProcessor::PostProcessor(const GlobalMainConfig& config)
  : config_(config.post_process),
    max_kappa_((config.vehicle.min_turning_radius > 0.1) ? (1.0 / config.vehicle.min_turning_radius) : 0.2),
    wheelbase_((config.vehicle.wheelbase > 0.1) ? config.vehicle.wheelbase : 2.97) {}

PostProcessor::PostProcessor(const PostProcessConfig& config,
                             double wheelbase,
                             double min_turning_radius)
  : config_(config),
    max_kappa_((min_turning_radius > 0.1) ? (1.0 / min_turning_radius) : 0.2),
    wheelbase_((wheelbase > 0.1) ? wheelbase : 2.97) {}

bool PostProcessor::process(GlobalPlanningResult& result,
                            const GlobalMap& map,
                            double start_vel,
                            bool request_smoothing,
                            bool request_resampling,
                            bool request_profiling) {
  return runPipeline(result,
                     &map,
                     start_vel,
                     request_smoothing,
                     request_resampling,
                     request_profiling);
}

bool PostProcessor::process(LocalPlanningResult& result,
                            double start_vel,
                            bool request_smoothing,
                            bool request_resampling,
                            bool request_profiling) {
  return runPipeline(result,
                     nullptr,
                     start_vel,
                     request_smoothing,
                     request_resampling,
                     request_profiling);
}

template <typename ResultT>
bool PostProcessor::runPipeline(ResultT& result,
                                const GlobalMap* map,
                                double start_vel,
                                bool request_smoothing,
                                bool request_resampling,
                                bool request_profiling) {
  using Clock = std::chrono::steady_clock;

  const bool run_smoothing = request_smoothing && config_.smoother.enabled;
  const bool run_resampling = request_resampling && config_.resampler.enabled;
  const bool run_profiling = request_profiling && config_.profiler.enabled;
  const bool run_final_resampling = run_resampling && run_smoothing;

  // 1. Pre-resampling/densify before smoothing
  if (run_resampling) {
    const auto start = Clock::now();
    Path resampled_path;
    if (resample(result.path, resampled_path, result.logs)) {
      result.path = std::move(resampled_path);
    }
    result.resampling_time = std::chrono::duration<double>(Clock::now() - start).count();
  }

  // 2. Smoothing
  if (run_smoothing) {
    if (config_.smoother.collision_check_enabled && !map) {
      result.logs.push_back({"ERROR", "PostProcessor collision-aware smoothing requested without a map."});
      return false;
    }
    const auto start = Clock::now();
    smooth(result.path, config_.smoother.collision_check_enabled ? map : nullptr, result.logs);
    result.smoothing_time = std::chrono::duration<double>(Clock::now() - start).count();
  }

  // 3. Final resampling after smoothing so the published/profiled path has stable spacing.
  if (run_final_resampling) {
    const auto start = Clock::now();
    Path resampled_path;
    if (resample(result.path, resampled_path, result.logs)) {
      result.path = std::move(resampled_path);
    }
    result.resampling_time += std::chrono::duration<double>(Clock::now() - start).count();
  }

  // Update geometry properties (theta, kappa, s) once on the finalized path
  updateGeometryProperties(result.path);

  // 4. Velocity profiling
  if (run_profiling) {
    const auto start = Clock::now();
    profile(result.path, start_vel, result.logs);
    result.profiling_time = std::chrono::duration<double>(Clock::now() - start).count();
  } else {
    for (auto& pt : result.path) {
      pt.v = 0.0;
      pt.a = 0.0;
    }
  }

  if (!config_.validator.enabled) {
    result.diagnostic = TrajectoryDiagnostic();
    return true;
  }

  TrajectoryValidator validator(config_, max_kappa_, wheelbase_);
  TrajectoryDiagnostic diag = validator.validate(result.path, result.logs);
  result.diagnostic = diag;
  return diag.is_valid;
}

template bool PostProcessor::runPipeline<GlobalPlanningResult>(
    GlobalPlanningResult& result,
    const GlobalMap* map,
    double start_vel,
    bool request_smoothing,
    bool request_resampling,
    bool request_profiling);

template bool PostProcessor::runPipeline<LocalPlanningResult>(
    LocalPlanningResult& result,
    const GlobalMap* map,
    double start_vel,
    bool request_smoothing,
    bool request_resampling,
    bool request_profiling);

bool PostProcessor::smooth(Path& path, const GlobalMap* map, std::vector<std::pair<std::string, std::string>>& logs) {
  if (path.size() < 3) {
    return true;
  }

  // 1. Split path into segments of uniform direction
  std::vector<std::pair<size_t, size_t>> segments = splitIntoSegments(path);

  const double w_data = config_.smoother.weight_data;
  const double w_smooth = config_.smoother.weight_smooth;
  const double tolerance = config_.smoother.tolerance;
  const int max_iter = config_.smoother.max_iterations;

  // 2. Smooth each segment independently
  for (const auto& seg : segments) {
    size_t seg_start = seg.first;
    size_t seg_end = seg.second;
    int seg_len = static_cast<int>(seg_end - seg_start + 1);
    if (seg_len < 3) continue; // Too short to smooth internal points

    Path original_segment(path.begin() + seg_start, path.begin() + seg_end + 1);

    // Cache distances between adjacent points in the segment
    std::vector<double> h(seg_len - 1);
    for (int i = 0; i < seg_len - 1; ++i) {
      h[i] = dist(original_segment[i], original_segment[i+1]);
    }

    double current_w_smooth = w_smooth;
    bool smooth_success = false;
    constexpr int MAX_TRIALS = 3;

    for (int trial = 0; trial < MAX_TRIALS; ++trial) {
      // Initialize this trial's path with the original segment
      for (int i = 0; i < seg_len; ++i) {
        path[seg_start + i] = original_segment[i];
      }

      double change = tolerance + 1.0;
      int iter = 0;

      while (change >= tolerance && iter < max_iter) {
        iter++;
        change = 0.0;

        // Keep endpoints (seg_start and seg_end) fixed
        for (int i = 1; i < seg_len - 1; ++i) {
          size_t global_idx = seg_start + i;
          const double x_orig = original_segment[i].x;
          const double y_orig = original_segment[i].y;

          double& x = path[global_idx].x;
          double& y = path[global_idx].y;

          const double prev_x = x;
          const double prev_y = y;

          // Distance-weighted Laplace smoothing using pre-cached static distances
          const double h1 = h[i-1];
          const double h2 = h[i];
          const double sum_h = h1 + h2;
          const double x_target = (sum_h > 1e-5) ? (h2 * path[global_idx - 1].x + h1 * path[global_idx + 1].x) / sum_h
                                                 : 0.5 * (path[global_idx - 1].x + path[global_idx + 1].x);
          const double y_target = (sum_h > 1e-5) ? (h2 * path[global_idx - 1].y + h1 * path[global_idx + 1].y) / sum_h
                                                 : 0.5 * (path[global_idx - 1].y + path[global_idx + 1].y);

          x = x + w_data * (x_orig - x) + current_w_smooth * 2.0 * (x_target - x);
          y = y + w_data * (y_orig - y) + current_w_smooth * 2.0 * (y_target - y);
          change += std::abs(x - prev_x) + std::abs(y - prev_y);
        }
      }

      // Calculate ONLY Yaw (theta) for this segment to check collision and loop/twist constraint
      for (int i = 0; i < seg_len; ++i) {
        size_t global_idx = seg_start + i;
        double dx = 0.0, dy = 0.0;
        if (i == 0) {
          dx = path[global_idx + 1].x - path[global_idx].x;
          dy = path[global_idx + 1].y - path[global_idx].y;
        } else if (i == seg_len - 1) {
          dx = path[global_idx].x - path[global_idx - 1].x;
          dy = path[global_idx].y - path[global_idx - 1].y;
        } else {
          dx = path[global_idx + 1].x - path[global_idx - 1].x;
          dy = path[global_idx + 1].y - path[global_idx - 1].y;
        }

        if (dist(dx, dy) > 1e-3) {
          double angle = std::atan2(dy, dx);
          if (path[global_idx].direction == -1) {
            angle = wrap_to_pi(angle + PI);
          }
          path[global_idx].theta = angle;
        }
      }

      // Check collision ONLY for internal points (excluding fixed endpoints)
      bool trial_ok = true;
      if (map) {
        for (int i = 1; i < seg_len - 1; ++i) {
          size_t global_idx = seg_start + i;
          if (map->checkCollision(path[global_idx].x, path[global_idx].y, path[global_idx].theta)) {
            trial_ok = false;
            break;
          }
        }
      }

      // Check loops/twists for the entire segment
      if (trial_ok) {
        for (int i = 1; i < seg_len; ++i) {
          size_t global_idx = seg_start + i;
          double angle_diff = std::abs(wrap_to_pi(path[global_idx].theta - path[global_idx - 1].theta));
          if (angle_diff > 1.22) { // 1.22 rad (approx. 70 deg)
            trial_ok = false;
            break;
          }
        }
      }

      if (trial_ok) {
        smooth_success = true;
        break; // Successfully smoothed this segment without violations
      }

      // If violations found, scale down the smoothing weight and retry
      current_w_smooth *= 0.5;
    }

    if (!smooth_success) {
      logs.push_back({"WARN", "Smoother failed to find a valid path after multiple attempts. Rolling back to original segment."});
      for (int i = 0; i < seg_len; ++i) {
        path[seg_start + i] = original_segment[i];
      }
    }
  }

  return true;
}

bool PostProcessor::resample(const Path& path, Path& resampled_path, std::vector<std::pair<std::string, std::string>>& logs) {
  if (path.size() < 2) {
    resampled_path = path;
    return true;
  }

  double step_size = config_.resampler.resolution;
  if (step_size <= 1e-4) {
    step_size = 0.1;
  }

  double total_dist = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    total_dist += dist(path[i - 1], path[i]);
  }

  resampled_path.clear();
  resampled_path.reserve(static_cast<size_t>(total_dist / step_size) + path.size());

  const auto segments = splitIntoSegments(path);
  for (const auto& seg : segments) {
    Path segment(path.begin() + static_cast<std::ptrdiff_t>(seg.first),
                 path.begin() + static_cast<std::ptrdiff_t>(seg.second + 1));
    Path segment_resampled;
    if (!resampleSegment(segment, segment_resampled, logs)) {
      return false;
    }
    resampled_path.insert(resampled_path.end(), segment_resampled.begin(), segment_resampled.end());
  }

  // 1. Recalculate accumulated distance s
  double s = 0.0;
  if (!resampled_path.empty()) resampled_path.front().s = 0.0;
  for (size_t i = 1; i < resampled_path.size(); ++i) {
    s += dist(resampled_path[i - 1], resampled_path[i]);
    resampled_path[i].s = s;
  }

  return true;
}

bool PostProcessor::resampleSegment(const Path& input_segment, Path& output_path, std::vector<std::pair<std::string, std::string>>& logs) {
  if (input_segment.empty()) {
    logs.push_back({"WARN", "resampleSegment bypassed: input segment is empty."});
    return false;
  }

  double step_size = config_.resampler.resolution;
  if (step_size <= 1e-4) {
    step_size = 0.1;
  }

  std::vector<double> s_vals;
  s_vals.reserve(input_segment.size());
  s_vals.push_back(0.0);

  double segment_len = 0.0;
  for (size_t i = 1; i < input_segment.size(); ++i) {
    segment_len += dist(input_segment[i - 1], input_segment[i]);
    s_vals.push_back(segment_len);
  }

  if (segment_len < 1e-6) {
    output_path.push_back(input_segment.back());
    output_path.back().s = 0.0;
    return true;
  }

  const int num_steps = static_cast<int>(std::floor(segment_len / step_size));
  const int direction = input_segment.front().direction < 0 ? -1 : 1;

  size_t current_idx = 0;
  for (int i = 0; i <= num_steps; ++i) {
    const double s_req = i * step_size;
    while (current_idx + 1 < s_vals.size() && s_vals[current_idx + 1] < s_req) {
      ++current_idx;
    }
    if (current_idx + 1 >= s_vals.size()) {
      break;
    }

    const double s0 = s_vals[current_idx];
    const double s1 = s_vals[current_idx + 1];
    const double denominator = s1 - s0;
    const double ratio = std::abs(denominator) > 1e-6 ? (s_req - s0) / denominator : 0.0;

    const auto& p0 = input_segment[current_idx];
    const auto& p1 = input_segment[current_idx + 1];

    PathPoint p;
    p.direction = direction;
    p.s = s_req;
    p.x = p0.x + ratio * (p1.x - p0.x);
    p.y = p0.y + ratio * (p1.y - p0.y);
    output_path.push_back(p);
  }

  const auto& last_pt = input_segment.back();
  if (output_path.empty()) {
    logs.push_back({"WARN", "resampleSegment: output path is empty after interpolation. Using last point."});
    output_path.push_back(last_pt);
  } else if (dist(output_path.back(), last_pt) > 1e-3) {
    PathPoint p_last = last_pt;
    p_last.s = segment_len;
    p_last.direction = direction;
    output_path.push_back(p_last);
  } else {
    output_path.back() = last_pt;
    output_path.back().s = segment_len;
    output_path.back().direction = direction;
  }

  return true;
}

bool PostProcessor::profile(Path& path, double start_vel, std::vector<std::pair<std::string, std::string>>& logs) {
  if (path.empty()) {
    logs.push_back({"WARN", "Velocity profiling bypassed: Input path is empty."});
    return false;
  }

  if (path.size() < 2) {
    path.back().v = config_.profiler.goal_vel;
    path.back().a = 0.0;
    path.back().t = 0.0;
    return true;
  }

  KinematicLimits limits;
  limits.max_vel = config_.profiler.max_vel;
  limits.max_accel = config_.profiler.max_accel;
  limits.max_decel = config_.profiler.max_decel;
  limits.max_jerk = config_.profiler.max_jerk;
  limits.max_steer_vel = config_.profiler.max_steer_vel;
  limits.max_lat_acc = config_.profiler.max_lat_acc;
  limits.min_vel_denom = config_.profiler.min_velocity_denominator;

  // Split path into segments of uniform direction
  std::vector<std::pair<size_t, size_t>> segments = splitIntoSegments(path);

  double current_time_offset = 0.0;

  for (size_t j = 0; j < segments.size(); ++j) {
    size_t seg_start = segments[j].first;
    size_t seg_end = segments[j].second;
    size_t seg_len = seg_end - seg_start + 1;

    // Create a temporary path for this segment
    Path seg_path(path.begin() + seg_start, path.begin() + seg_end + 1);

    // Determine target start and goal velocities for this segment
    double seg_start_v = (j == 0) ? start_vel : 0.0;
    double seg_goal_v = (j == segments.size() - 1) ? config_.profiler.goal_vel : 0.0;

    // Profile the segment
    profileKinematicPass(seg_path, seg_start_v, 0.0, seg_goal_v, 0.0, limits, logs);

    // If this is not the first segment, calculate transition time from the end of the previous segment
    if (j > 0) {
      size_t prev_end = segments[j - 1].second;
      double ds = dist(path[prev_end], path[seg_start]);
      // Use configurable gear_shift_duration for the stationary transition time at the cusp
      double dt = (ds > 1e-6) ? ds / 0.1 : config_.profiler.gear_shift_duration;
      current_time_offset = path[prev_end].t + dt;
    }

    // Copy profiled values back and apply time offset
    for (size_t i = 0; i < seg_len; ++i) {
      size_t global_idx = seg_start + i;
      path[global_idx].v = seg_path[i].v;
      path[global_idx].a = seg_path[i].a;
      path[global_idx].t = seg_path[i].t + current_time_offset;
    }
  }

  return true;
}

void PostProcessor::profileKinematicPass(Path& path, double start_v, double start_a,
                                         double goal_v, double goal_a, const KinematicLimits& limits,
                                         std::vector<std::pair<std::string, std::string>>& logs,
                                         bool limit_by_input_v) {
  if (path.empty()) return;
  const int n = static_cast<int>(path.size());
  const double max_accel = std::abs(limits.max_accel);
  const double max_decel = -std::abs(limits.max_decel);
  const double max_jerk = std::abs(limits.max_jerk);

  std::vector<double> v_static(n);
  for (int i = 0; i < n; ++i) {
    double v_curve = (std::abs(path[i].kappa) > 1e-3)
      ? std::sqrt(limits.max_lat_acc / std::abs(path[i].kappa)) : limits.max_vel;
    v_static[i] = std::min(limits.max_vel, v_curve);
    if (limit_by_input_v) {
      v_static[i] = std::min(v_static[i], path[i].v);
    }
  }

  // Cap velocity at adjacent points in high steer rate zones to satisfy Steering Velocity limit
  const double max_steer_vel = limits.max_steer_vel;
  for (int i = 1; i < n; ++i) {
    double ds = dist(path[i-1], path[i]);
    double phi_i = std::atan(wheelbase_ * path[i].kappa);
    double phi_prev = std::atan(wheelbase_ * path[i-1].kappa);
    double dphi_ds = (ds > 1e-4) ? std::abs(phi_i - phi_prev) / ds : 0.0;
    if (dphi_ds > 1e-4) {
      // Apply 0.95 safety margin on steering velocity limit to absorb profile numerical noises
      double v_steer = (max_steer_vel * 0.95) / dphi_ds;
      v_static[i] = std::min(v_static[i], v_steer);
      v_static[i-1] = std::min(v_static[i-1], v_steer);
    }
  }

  // Smooth v_static to eliminate any high-frequency speed limit oscillations
  if (n > 2) {
    int half_w = std::min(2, static_cast<int>(n - 1) / 2); // 5-point window
    for (int iter = 0; iter < 2; ++iter) { // 2 passes
      std::vector<double> smoothed_v_static = v_static;
      for (int i = 1; i < n - 1; ++i) {
        if (v_static[i] < 0.1 || v_static[i-1] < 0.1 || v_static[i+1] < 0.1) {
          continue;
        }
        double sum = 0.0;
        int count = 0;
        for (int w = -half_w; w <= half_w; ++w) {
          int idx = i + w;
          if (idx >= 0 && idx < n) {
            sum += v_static[idx];
            count++;
          }
        }
        smoothed_v_static[i] = sum / count;
      }
      v_static = std::move(smoothed_v_static);
    }
  }

  std::vector<double> v_fwd(n), a_fwd(n);
  v_fwd[0] = std::max(0.0, start_v);
  a_fwd[0] = std::clamp(start_a, max_decel, max_accel);
  for (int i = 1; i < n; ++i) {
    const double ds = dist(path[i-1], path[i]);
    const double dt = (ds > 1e-6) ? ds / std::max(limits.min_vel_denom, v_fwd[i-1]) : limits.min_vel_denom;
    const double a_lim = std::min(max_accel, a_fwd[i-1] + max_jerk * dt);
    const double v_next_sq = v_fwd[i-1]*v_fwd[i-1] + 2.0*a_lim*ds;
    v_fwd[i] = std::sqrt(std::max(0.0, v_next_sq));
    a_fwd[i] = std::clamp((ds > 1e-6) ? (v_fwd[i]*v_fwd[i] - v_fwd[i-1]*v_fwd[i-1]) / (2.0 * ds) : a_fwd[i-1], max_decel, max_accel);
  }

  std::vector<double> v_bwd(n), a_bwd(n);
  v_bwd[n-1] = std::max(0.0, goal_v);
  a_bwd[n-1] = std::clamp(goal_a, max_decel, max_accel);
  for (int i = n-2; i >= 0; --i) {
    const double ds = dist(path[i], path[i+1]);
    const double dt = (ds > 1e-6) ? ds / std::max(limits.min_vel_denom, v_bwd[i+1]) : limits.min_vel_denom;
    const double a_lim = std::max(max_decel, a_bwd[i+1] - max_jerk * dt);
    const double v_next_sq = v_bwd[i+1]*v_bwd[i+1] - 2.0*a_lim*ds;
    v_bwd[i] = std::sqrt(std::max(0.0, v_next_sq));
    a_bwd[i] = std::clamp((ds > 1e-6) ? (v_bwd[i+1]*v_bwd[i+1] - v_bwd[i]*v_bwd[i]) / (2.0 * ds) : a_bwd[i+1], max_decel, max_accel);
  }

  if (v_bwd[0] < start_v - 0.5) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Kinematic unreachability detected in segment: start velocity mismatch (v_bwd[0]: %.2f m/s vs start_v: %.2f m/s) exceeds 0.5 m/s threshold.",
      v_bwd[0], start_v);
    logs.push_back({"WARN", std::string(buf)});
  }

  std::vector<double> v_final(n);
  for (int i = 0; i < n; ++i) {
    v_final[i] = std::min({v_fwd[i], v_bwd[i], v_static[i]});
  }

  // 1. Smooth the velocity profile to suppress high-frequency speed variations (which cause Jerk spikes)
  if (n > 2) {
    std::vector<double> smoothed_v = v_final;
    // 5 passes of 3-point moving average to stronger filter Jerk high frequency noise
    for (int iter = 0; iter < 5; ++iter) {
      std::vector<double> temp = smoothed_v;
      for (int i = 1; i < n - 1; ++i) {
        // Keep Cusp/Goal/Start velocity 0.0 constraint preserved (anchor points)
        if (temp[i] < 0.1) {
          continue;
        }
        // Smooth non-zero points, allowing them to blend with neighboring zero velocities smoothly
        smoothed_v[i] = (temp[i-1] + temp[i] + temp[i+1]) / 3.0;
      }
    }
    v_final = std::move(smoothed_v);
  }

  // 2. Re-calculate acceleration and timestamps to maintain 100% physical consistency (a = dv/dt, dt = ds/v)
  for (int i = 0; i < n; ++i) {
    path[i].v = v_final[i];
    if (i == 0) {
      path[i].a = a_fwd[0];
      path[i].t = 0.0;
    } else {
      const double ds = dist(path[i-1], path[i]);
      const double dt = (ds > 1e-6) ? ds / std::max(limits.min_vel_denom, (path[i].v + path[i-1].v) / 2.0) : limits.min_vel_denom;
      // Apply 0.01s denominator guard to prevent division by extremely small dt (which causes acceleration to spike)
      const double acc_dt = std::max(0.01, dt);

      // Calculate raw acceleration from speed difference
      double raw_a = (path[i].v - path[i-1].v) / acc_dt;
      raw_a = std::clamp(raw_a, max_decel, max_accel);

      // Enforce the maximum Jerk constraint directly on the calculated acceleration profile
      if (path[i].direction == path[i-1].direction) {
        const double max_a_change = max_jerk * acc_dt;
        raw_a = std::clamp(raw_a, path[i-1].a - max_a_change, path[i-1].a + max_a_change);
      }

      path[i].a = raw_a;
      path[i].t = path[i-1].t + dt;
    }
  }
}


std::vector<std::pair<size_t, size_t>> PostProcessor::splitIntoSegments(const Path& path) const {
  std::vector<std::pair<size_t, size_t>> segments;
  if (path.empty()) return segments;
  size_t start_idx = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    if (path[i].direction != path[start_idx].direction) {
      segments.push_back({start_idx, i - 1});
      start_idx = i;
    }
  }
  segments.push_back({start_idx, path.size() - 1});
  return segments;
}


// ── TrajectoryValidator Implementation ────────────────────────────────

TrajectoryValidator::TrajectoryValidator(const PostProcessConfig& config, double max_kappa, double wheelbase)
  : config_(config), max_kappa_(max_kappa), wheelbase_(wheelbase) {}

TrajectoryDiagnostic TrajectoryValidator::validate(const Path& path, std::vector<std::pair<std::string, std::string>>& logs) const {
  TrajectoryDiagnostic diag;
  if (path.empty()) {
    return diag;
  }

  // Initialize to clean slate (no violations) for a non-empty trajectory
  diag.is_valid = true;
  diag.curvature_ok = true;
  diag.yaw_ok = true;
  diag.time_ok = true;
  diag.vel_ok = true;
  diag.acc_ok = true;
  diag.jerk_ok = true;
  diag.steer_vel_ok = true;

  diag.curv_violations = 0;
  diag.yaw_violations = 0;
  diag.time_violations = 0;
  diag.vel_violations = 0;
  diag.acc_violations = 0;
  diag.jerk_violations = 0;
  diag.steer_vel_violations = 0;

  diag.max_curv_violation = 0.0;
  diag.max_yaw_error_rad = 0.0;
  diag.max_time_error_sec = 0.0;
  diag.max_vel_violation = 0.0;
  diag.max_acc_violation = 0.0;
  diag.max_jerk_violation = 0.0;
  diag.max_steer_vel_violation = 0.0;

  validateCurvature(path, diag, logs);
  validateYawAlignment(path, diag, logs);
  validateTimestamps(path, diag, logs);
  validateVelocity(path, diag, logs);
  validateAcceleration(path, diag, logs);
  validateJerk(path, diag, logs);
  validateSteeringVelocity(path, diag, logs);

  diag.is_valid = diag.curvature_ok && diag.yaw_ok && diag.time_ok &&
                  diag.vel_ok && diag.acc_ok && diag.jerk_ok && diag.steer_vel_ok;
  return diag;
}

void TrajectoryValidator::validateCurvature(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  checkLimit(path, diag.curvature_ok, diag.curv_violations, diag.max_curv_violation,
             max_kappa_, 1e-3, [&](size_t i) { return std::abs(path[i].kappa); });

  if (!diag.curvature_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Curvature check failed: %d points exceeded max curvature limit (max_kappa: %.4f). Max violation: %.4f rad/m.",
      diag.curv_violations, max_kappa_, diag.max_curv_violation);
    logs.push_back({"WARN", std::string(buf)});
  }
}

void TrajectoryValidator::validateYawAlignment(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double yaw_tol = config_.validator.yaw_tolerance_rad;

  for (size_t i = 0; i < path.size(); ++i) {
    // Exclude points near direction changes (cusp zones)
    if (isNearCusp(path, i, 0.2)) continue;

    double dx = 0.0, dy = 0.0;
    if (i == 0) {
      if (path.size() > 1) {
        dx = path[1].x - path[0].x;
        dy = path[1].y - path[0].y;
      }
    } else if (i == path.size() - 1) {
      if (path.size() > 1) {
        dx = path[i].x - path[i-1].x;
        dy = path[i].y - path[i-1].y;
      }
    } else {
      dx = path[i+1].x - path[i-1].x;
      dy = path[i+1].y - path[i-1].y;
    }

    double ds = std::hypot(dx, dy);
    if (ds < 1e-3) continue;

    double expected_yaw = std::atan2(dy, dx);
    if (path[i].direction == -1) {
      expected_yaw = wrap_to_pi(expected_yaw + PI);
    }

    double diff = std::abs(wrap_to_pi(path[i].theta - expected_yaw));
    if (diff > yaw_tol) {
      diag.yaw_ok = false;
      diag.yaw_violations++;
      diag.max_yaw_error_rad = std::max(diag.max_yaw_error_rad, diff);
    }
  }

  if (!diag.yaw_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Yaw alignment check failed: %d points exceeded tolerance of %.1f deg. Max error: %.2f deg.",
      diag.yaw_violations, rad2deg(yaw_tol), rad2deg(diag.max_yaw_error_rad));
    logs.push_back({"WARN", std::string(buf)});
  }
}

void TrajectoryValidator::validateTimestamps(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double time_tol = config_.validator.time_tolerance_ms / 1000.0;

  double min_vel_denom = config_.profiler.min_velocity_denominator;
  if (min_vel_denom <= 1e-4) {
    min_vel_denom = 0.02; // default fallback
  }

  for (size_t i = 1; i < path.size(); ++i) {
    double dt = path[i].t - path[i-1].t;
    if (dt <= 1e-6) {
      diag.time_ok = false;
      diag.time_violations++;
      continue;
    }

    if (path[i].direction == path[i-1].direction) {
      double ds = dist(path[i-1], path[i]);
      double v_avg = (path[i].v + path[i-1].v) / 2.0;
      double expected_dt = (ds > 1e-6) ? ds / std::max(min_vel_denom, v_avg) : min_vel_denom;
      double err = std::abs(dt - expected_dt);
      if (err > time_tol) {
        diag.time_ok = false;
        diag.time_violations++;
        diag.max_time_error_sec = std::max(diag.max_time_error_sec, err);
      }
    }
  }

  if (!diag.time_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Timestamp consistency check failed: %d points had non-monotonic or inconsistent timestamps. Max time error: %.4f s.",
      diag.time_violations, diag.max_time_error_sec);
    logs.push_back({"WARN", std::string(buf)});
  }
}

void TrajectoryValidator::validateVelocity(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double max_vel = config_.profiler.max_vel;
  checkLimit(path, diag.vel_ok, diag.vel_violations, diag.max_vel_violation,
             max_vel, 0.01, [&](size_t i) { return std::abs(path[i].v); });

  if (!diag.vel_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Velocity check failed: %d points exceeded max velocity limit (max_vel: %.4f). Max violation: %.4f m/s.",
      diag.vel_violations, max_vel, diag.max_vel_violation);
    logs.push_back({"WARN", std::string(buf)});
  }
}

void TrajectoryValidator::validateAcceleration(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double max_accel = std::abs(config_.profiler.max_accel);
  const double max_decel = std::abs(config_.profiler.max_decel);

  checkLimit(path, diag.acc_ok, diag.acc_violations, diag.max_acc_violation,
             max_accel, 0.01, [&](size_t i) { return path[i].a * path[i].direction; });

  checkLimit(path, diag.acc_ok, diag.acc_violations, diag.max_acc_violation,
             max_decel, 0.01, [&](size_t i) { return -(path[i].a * path[i].direction); });

  if (!diag.acc_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Acceleration check failed: %d points exceeded limits (accel: %.4f, decel: %.4f). Max violation: %.4f m/s^2.",
      diag.acc_violations, max_accel, max_decel, diag.max_acc_violation);
    logs.push_back({"WARN", std::string(buf)});
  }
}

void TrajectoryValidator::validateJerk(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double max_jerk = config_.profiler.max_jerk;
  checkLimit(path, diag.jerk_ok, diag.jerk_violations, diag.max_jerk_violation,
             max_jerk, 0.01,
             [&](size_t i) {
               // Skip verification at segment transitions (Cusp) where direction changes
               if (path[i].direction != path[i-1].direction) return 0.0;
               if (std::abs(path[i].v) < 0.1 || std::abs(path[i-1].v) < 0.1) return 0.0;
               double dt = path[i].t - path[i-1].t;
               double acc_dt = std::max(0.01, dt); // Match the planner's denominator guard
               return (dt > 1e-6) ? std::abs(path[i].a - path[i-1].a) / acc_dt : 0.0;
             },
             1);

  if (!diag.jerk_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Jerk check failed: %d points exceeded max jerk limit (max_jerk: %.4f). Max violation: %.4f m/s^3.",
      diag.jerk_violations, max_jerk, diag.max_jerk_violation);
    logs.push_back({"WARN", std::string(buf)});
  }
}

void TrajectoryValidator::validateSteeringVelocity(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double max_steer_vel_limit = config_.profiler.max_steer_vel;
  checkLimit(path, diag.steer_vel_ok, diag.steer_vel_violations, diag.max_steer_vel_violation,
             max_steer_vel_limit, 0.1,
             [&](size_t i) {
               double dt = path[i].t - path[i-1].t;
               if (dt <= 1e-6) return 0.0;

               double phi_i = std::atan(wheelbase_ * path[i].kappa);
               double phi_prev = std::atan(wheelbase_ * path[i-1].kappa);

               if (path[i].direction != path[i-1].direction) {
                 // At segment transitions (Cusp), verify if the steer angle can rotate within the gear shift duration (dt)
                 return std::abs(phi_i - phi_prev) / dt;
               }

               if (std::abs(path[i].v) < 0.1 || std::abs(path[i-1].v) < 0.1) return 0.0;
               double acc_dt = std::max(0.01, dt); // Match the planner's denominator guard
               return std::abs(phi_i - phi_prev) / acc_dt;
             },
             1);

  if (!diag.steer_vel_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Steering Velocity check failed: %d points exceeded limit (max_steer_vel: %.4f rad/s). Max violation: %.4f rad/s.",
      diag.steer_vel_violations, max_steer_vel_limit, diag.max_steer_vel_violation);
    logs.push_back({"WARN", std::string(buf)});
  }
}

bool TrajectoryValidator::isNearCusp(const Path& path, size_t index, double radius_threshold) const {
  const double resolution = config_.resampler.resolution > 1e-4 ? config_.resampler.resolution : 0.1;
  const int search_window = std::max(3, static_cast<int>(std::round(0.3 / resolution)));

  for (int w = -search_window; w <= search_window; ++w) {
    int idx = static_cast<int>(index) + w;
    if (idx >= 0 && idx < static_cast<int>(path.size())) {
      if (path[idx].direction != path[index].direction) {
        double actual_dist = dist(path[index], path[idx]);
        if (actual_dist < radius_threshold) {
          return true;
        }
      }
    }
  }
  return false;
}

void PostProcessor::updateGeometryProperties(Path& path) const {
  if (path.empty()) return;

  // 1. Recalculate accumulated distance s
  double s = 0.0;
  path[0].s = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    s += dist(path[i - 1], path[i]);
    path[i].s = s;
  }

  // 2. Split path into segments of uniform direction
  const auto segments = splitIntoSegments(path);
  for (const auto& seg : segments) {
    const size_t seg_start = seg.first;
    const size_t seg_end = seg.second;
    const size_t seg_len = seg_end - seg_start + 1;
    if (seg_len == 0) continue;

    const int direction = path[seg_start].direction < 0 ? -1 : 1;

    // 2-1) Calculate heading (yaw) for this segment
    for (size_t i = 0; i < seg_len; ++i) {
      const size_t global_idx = seg_start + i;
      double dx = 0.0;
      double dy = 0.0;
      if (i == 0) {
        if (seg_len > 1) {
          dx = path[global_idx + 1].x - path[global_idx].x;
          dy = path[global_idx + 1].y - path[global_idx].y;
        }
      } else if (i == seg_len - 1) {
        if (seg_len > 1) {
          dx = path[global_idx].x - path[global_idx - 1].x;
          dy = path[global_idx].y - path[global_idx - 1].y;
        }
      } else {
        dx = path[global_idx + 1].x - path[global_idx - 1].x;
        dy = path[global_idx + 1].y - path[global_idx - 1].y;
      }

      if (std::hypot(dx, dy) > 1e-3) {
        double angle = std::atan2(dy, dx);
        if (direction == -1) {
          angle = wrap_to_pi(angle + PI);
        }
        path[global_idx].theta = angle;
      } else {
        path[global_idx].theta = (i > 0) ? path[global_idx - 1].theta : path[seg_start].theta;
      }
      path[global_idx].direction = direction;
    }

    // 2-2) Calculate curvature (kappa) for this segment
    for (size_t i = 0; i < seg_len; ++i) {
      const size_t global_idx = seg_start + i;
      if (i == 0 || i == seg_len - 1) {
        path[global_idx].kappa = 0.0;
      } else {
        const double ds1 = dist(path[global_idx - 1], path[global_idx]);
        const double ds2 = dist(path[global_idx], path[global_idx + 1]);
        path[global_idx].kappa = symmetricDiffCurvature(
            path[global_idx - 1].theta,
            path[global_idx].theta,
            path[global_idx + 1].theta,
            ds1,
            ds2);
      }
    }

    if (seg_len >= 2) {
      path[seg_start].kappa = path[seg_start + 1].kappa;
      path[seg_end].kappa = path[seg_end - 1].kappa;
    }

    // 2-3) Apply wide-window adaptive Moving Average Filter on Curvature (kappa) multiple times
    if (seg_len > 2) {
      std::vector<double> kappas(seg_len);
      for (size_t i = 0; i < seg_len; ++i) {
        kappas[i] = path[seg_start + i].kappa;
      }
      const int half_w = std::min(5, static_cast<int>(seg_len - 1) / 2);
      applyMovingAverage(kappas, half_w, 2);
      for (size_t i = 0; i < seg_len; ++i) {
        path[seg_start + i].kappa = kappas[i];
      }
    }
  }

  // 3. Cusp points (where direction changes) must have zero curvature for safety
  for (size_t i = 1; i < path.size(); ++i) {
    if (path[i - 1].direction != path[i].direction) {
      path[i - 1].kappa = 0.0;
      path[i].kappa = 0.0;
    }
  }
}

} // namespace carmaker_planning
