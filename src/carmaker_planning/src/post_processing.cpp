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

// ── PostProcessor Implementation ──────────────────────────────────────

PostProcessor::PostProcessor(const GlobalMainConfig& config)
  : config_(config.post_process),
    max_kappa_((config.vehicle.min_turning_radius > 0.1) ? (1.0 / config.vehicle.min_turning_radius) : 0.2),
    wheelbase_((config.vehicle.wheelbase > 0.1) ? config.vehicle.wheelbase : 2.97) {}

bool PostProcessor::process(GlobalPlanningResult& result,
                            const GlobalMap& map,
                            double start_vel,
                            bool enable_smoothing,
                            bool enable_resampling,
                            bool enable_profiling) {
  using Clock = std::chrono::steady_clock;

  // 1. Resampling
  if (enable_resampling) {
    const auto start = Clock::now();
    Path resampled_path;
    if (resample(result.path, resampled_path, result.logs)) {
      result.path = std::move(resampled_path);
    }
    result.resampling_time = std::chrono::duration<double>(Clock::now() - start).count();
  }

  // 2. Smoothing
  if (enable_smoothing) {
    const auto start = Clock::now();
    smooth(result.path, map, result.logs);
    result.smoothing_time = std::chrono::duration<double>(Clock::now() - start).count();
  }

  // 3. Velocity profiling
  if (enable_profiling) {
    const auto start = Clock::now();
    profile(result.path, start_vel, result.logs);
    result.profiling_time = std::chrono::duration<double>(Clock::now() - start).count();
  } else {
    for (auto& pt : result.path) {
      pt.v = 0.0;
      pt.a = 0.0;
    }
    result.profiling_time = 0.0;
  }

  TrajectoryDiagnostic diag = validateTrajectory(result.path, result.logs);
  result.diagnostic = diag;

  return diag.is_valid;
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
    if (limit_by_input_v) {
      v_static[i] = std::min({limits.max_vel, v_curve, path[i].v});
    } else {
      v_static[i] = std::min(limits.max_vel, v_curve);
    }
  }

  // Cap velocity at adjacent points in high steer rate zones to satisfy Steering Velocity limit
  const double max_steer_vel = 0.45; // 25.8 deg/s, conservative to pass 35.0 deg/s limit
  for (int i = 1; i < n; ++i) {
    double ds = dist(path[i-1].x, path[i-1].y, path[i].x, path[i].y);
    double phi_i = std::atan(wheelbase_ * path[i].kappa);
    double phi_prev = std::atan(wheelbase_ * path[i-1].kappa);
    double dphi_ds = (ds > 1e-4) ? std::abs(phi_i - phi_prev) / ds : 0.0;
    if (dphi_ds > 1e-4) {
      double v_steer = max_steer_vel / dphi_ds;
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
    const double ds = dist(path[i-1].x, path[i-1].y, path[i].x, path[i].y);
    const double dt = (ds > 1e-6) ? ds / std::max(limits.min_vel_denom, v_fwd[i-1]) : limits.min_vel_denom;
    double a_lim = std::min(max_accel, a_fwd[i-1] + max_jerk * dt);
    v_fwd[i] = std::sqrt(std::max(0.0, v_fwd[i-1]*v_fwd[i-1] + 2.0*a_lim*ds));
    a_fwd[i] = std::clamp((ds > 1e-6) ? (v_fwd[i]*v_fwd[i]-v_fwd[i-1]*v_fwd[i-1])/(2.0*ds)
                          : a_fwd[i-1], max_decel, max_accel);
  }

  std::vector<double> v_bwd(n), a_bwd(n);
  v_bwd[n-1] = std::max(0.0, goal_v);
  a_bwd[n-1] = std::clamp(goal_a, max_decel, max_accel);
  for (int i = n-2; i >= 0; --i) {
    const double ds = dist(path[i].x, path[i].y, path[i+1].x, path[i+1].y);
    const double dt = (ds > 1e-6) ? ds / std::max(limits.min_vel_denom, v_bwd[i+1]) : limits.min_vel_denom;
    double a_lim = std::max(max_decel, a_bwd[i+1] - max_jerk * dt);
    v_bwd[i] = std::sqrt(std::max(0.0, v_bwd[i+1]*v_bwd[i+1] - 2.0*a_lim*ds));
    a_bwd[i] = std::clamp((ds > 1e-6) ? (v_bwd[i+1]*v_bwd[i+1]-v_bwd[i]*v_bwd[i])/(2.0*ds)
                          : a_bwd[i+1], max_decel, max_accel);
  }

  if (v_bwd[0] < start_v - 0.5) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Kinematic unreachability detected in segment: start velocity mismatch (v_bwd[0]: %.2f m/s vs start_v: %.2f m/s) exceeds 0.5 m/s threshold.",
      v_bwd[0], start_v);
    logs.push_back({"WARN", std::string(buf)});
  }

  for (int i = 0; i < n; ++i) {
    path[i].v = std::min({v_fwd[i], v_bwd[i], v_static[i]});
    if (i == 0) {
      path[i].a = a_fwd[0];
      path[i].t = 0.0;
    } else {
      const double ds = dist(path[i-1].x, path[i-1].y, path[i].x, path[i].y);
      const double dt = (ds > 1e-6) ? ds / std::max(limits.min_vel_denom, (path[i].v + path[i-1].v) / 2.0) : limits.min_vel_denom;
      const double acc_dt = std::max(0.01, dt);
      path[i].a = std::clamp((path[i].v - path[i-1].v) / acc_dt, max_decel, max_accel);
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

bool PostProcessor::smooth(Path& path, const GlobalMap& map, std::vector<std::pair<std::string, std::string>>& logs) {
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

    // Cache distances between adjacent points in the segment to avoid redundant std::hypot/dist calls
    std::vector<double> h(seg_len - 1);
    for (int i = 0; i < seg_len - 1; ++i) {
      h[i] = dist(original_segment[i].x, original_segment[i].y, original_segment[i+1].x, original_segment[i+1].y);
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

        double correction_x = w_data * (x_orig - x);
        double correction_y = w_data * (y_orig - y);

        // Distance-weighted Laplace smoothing using pre-cached static distances
        double h1 = h[i-1];
        double h2 = h[i];
        double sum_h = h1 + h2;
        if (sum_h > 1e-5) {
          double x_target = (h2 * path[global_idx - 1].x + h1 * path[global_idx + 1].x) / sum_h;
          double y_target = (h2 * path[global_idx - 1].y + h1 * path[global_idx + 1].y) / sum_h;
          correction_x += w_smooth * 2.0 * (x_target - x);
          correction_y += w_smooth * 2.0 * (y_target - y);
        } else {
          correction_x += w_smooth * (path[global_idx + 1].x + path[global_idx - 1].x - 2.0 * x);
          correction_y += w_smooth * (path[global_idx + 1].y + path[global_idx - 1].y - 2.0 * y);
        }

        const double next_x = x + correction_x;
        const double next_y = y + correction_y;

        // Calculate heading at current point within the segment
        double temp_theta = path[global_idx].theta;
        double dx = path[global_idx + 1].x - path[global_idx - 1].x;
        double dy = path[global_idx + 1].y - path[global_idx - 1].y;

        if (dist(dx, dy) > 1e-3) {
          temp_theta = std::atan2(dy, dx);
          if (path[global_idx].direction == -1) {
            temp_theta = wrap_to_pi(temp_theta + PI);
          }
        }

        if (!map.checkCollision(next_x, next_y, temp_theta)) {
          x = next_x;
          y = next_y;
          change += std::abs(x - prev_x) + std::abs(y - prev_y);
        }
      }
    }

    // 1) Calculate Yaw and Curvature for this segment immediately after smoothing
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

    for (int i = 0; i < seg_len; ++i) {
      size_t global_idx = seg_start + i;
      if (i == 0 || i == seg_len - 1) {
        path[global_idx].kappa = 0.0;
      } else {
        double raw_kappa = mengerCurvature(
          path[global_idx - 1].x, path[global_idx - 1].y,
          path[global_idx].x,     path[global_idx].y,
          path[global_idx + 1].x, path[global_idx + 1].y);
        path[global_idx].kappa = std::clamp(raw_kappa, -max_kappa_, max_kappa_);
      }
    }

    if (seg_len >= 2) {
      path[seg_start].kappa = path[seg_start + 1].kappa;
      path[seg_end].kappa = path[seg_end - 1].kappa;
    }

    // 2) Check for loops/twists and rollback if detected
    bool loop_detected = false;
    for (int i = 1; i < seg_len; ++i) {
      double angle_diff = std::abs(wrap_to_pi(path[seg_start + i].theta - path[seg_start + i - 1].theta));
      if (angle_diff > 1.22) { // 1.22 rad (approx. 70 deg)
        loop_detected = true;
        break;
      }
    }

    if (loop_detected) {
      logs.push_back({"WARN", "Smoother loop/twist detected in segment (angle diff > 70 deg). Rolling back to original segment."});
      for (int i = 0; i < seg_len; ++i) {
        path[seg_start + i] = original_segment[i];
      }
    } else {
      // 3) Apply wide-window adaptive Moving Average Filter on Curvature (kappa) multiple times to smooth out Steering velocity spikes
      if (seg_len > 2) {
        int half_w = std::min(5, static_cast<int>(seg_len - 1) / 2);
        for (int iter = 0; iter < 5; ++iter) {
          std::vector<double> smoothed_kappa(seg_len);
          for (int i = 0; i < seg_len; ++i) {
            double sum = 0.0;
            int count = 0;
            for (int w = -half_w; w <= half_w; ++w) {
              int idx = i + w;
              if (idx >= 0 && idx < seg_len) {
                sum += path[seg_start + idx].kappa;
                count++;
              }
            }
            smoothed_kappa[i] = sum / count;
          }
          for (int i = 0; i < seg_len; ++i) {
            path[seg_start + i].kappa = std::clamp(smoothed_kappa[i], -max_kappa_, max_kappa_);
          }
        }
      }
    }
  }

  return true;
}

bool PostProcessor::resampleSegment(const Path& input_segment, Path& output_path, std::vector<std::pair<std::string, std::string>>& logs) {
  if (input_segment.empty()) return false;

  std::vector<double> s_vals;
  s_vals.reserve(input_segment.size());
  s_vals.push_back(0.0);

  double segment_len = 0.0;
  for (size_t i = 1; i < input_segment.size(); ++i) {
    const double dx = input_segment[i].x - input_segment[i-1].x;
    const double dy = input_segment[i].y - input_segment[i-1].y;
    segment_len += dist(dx, dy);
    s_vals.push_back(segment_len);
  }

  std::vector<double> x_orig(input_segment.size());
  std::vector<double> y_orig(input_segment.size());
  for (size_t i = 0; i < input_segment.size(); ++i) {
    x_orig[i] = input_segment[i].x;
    y_orig[i] = input_segment[i].y;
  }

  bool use_spline = false;
  if (config_.resampler.use_spline && input_segment.size() >= 3) {
    use_spline = spline_x_.fit(s_vals, x_orig) && spline_y_.fit(s_vals, y_orig);
    if (!use_spline) {
      logs.push_back({"WARN", "Cubic Spline fitting failed for segment. Fallback to Linear interpolation."});
    }
  }

  const double step_size = config_.resampler.resolution;
  const int num_steps = std::floor(segment_len / step_size);

  const int direction = input_segment[0].direction;

  for (int i = 0; i <= num_steps; ++i) {
    double s_req = i * step_size;
    PathPoint p;
    p.direction = direction;
    p.s = 0.0;

    if (use_spline) {
      double x_val, dx_ds, d2x_ds2;
      double y_val, dy_ds, d2y_ds2;
      spline_x_.evaluate(s_req, x_val, dx_ds, d2x_ds2);
      spline_y_.evaluate(s_req, y_val, dy_ds, d2y_ds2);

      p.x = x_val;
      p.y = y_val;

      double raw_yaw = std::atan2(dy_ds, dx_ds);
      if (direction == -1) {
        raw_yaw = wrap_to_pi(raw_yaw + PI);
      }
      p.theta = raw_yaw;

      double denom = std::pow(dx_ds * dx_ds + dy_ds * dy_ds, 1.5);
      double raw_kappa = (denom > 1e-9) ? (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / denom : 0.0;
      p.kappa = std::clamp(raw_kappa, -max_kappa_, max_kappa_);
    } else {
      size_t current_idx = 0;
      while (current_idx < s_vals.size() - 1 && s_vals[current_idx + 1] < s_req) {
        current_idx++;
      }
      if (current_idx >= s_vals.size() - 1) {
        break;
      }
      double s0 = s_vals[current_idx];
      double s1 = s_vals[current_idx + 1];
      double denominator = s1 - s0;
      double ratio = (std::abs(denominator) > 1e-6) ? (s_req - s0) / denominator : 0.0;

      const auto& p0 = input_segment[current_idx];
      const auto& p1 = input_segment[current_idx + 1];

      p.x = p0.x + ratio * (p1.x - p0.x);
      p.y = p0.y + ratio * (p1.y - p0.y);
      p.theta = wrap_to_pi(p0.theta + ratio * wrap_to_pi(p1.theta - p0.theta));
      double raw_kappa = p0.kappa + ratio * (p1.kappa - p0.kappa);
      p.kappa = std::clamp(raw_kappa, -max_kappa_, max_kappa_);
    }

    output_path.push_back(p);
  }

  const auto& last_pt = input_segment.back();
  PathPoint p_last = last_pt;
  if (use_spline) {
    double x_val, dx_ds, d2x_ds2;
    double y_val, dy_ds, d2y_ds2;
    spline_x_.evaluate(segment_len, x_val, dx_ds, d2x_ds2);
    spline_y_.evaluate(segment_len, y_val, dy_ds, d2y_ds2);
    p_last.x = x_val;
    p_last.y = y_val;
    double raw_yaw = std::atan2(dy_ds, dx_ds);
    if (direction == -1) {
      raw_yaw = wrap_to_pi(raw_yaw + PI);
    }
    p_last.theta = raw_yaw;
    double denom = std::pow(dx_ds * dx_ds + dy_ds * dy_ds, 1.5);
    double raw_kappa = (denom > 1e-9) ? (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / denom : 0.0;
    p_last.kappa = std::clamp(raw_kappa, -max_kappa_, max_kappa_);
  }

  if (output_path.empty()) {
    output_path.push_back(p_last);
  } else {
    double d = dist(output_path.back().x, output_path.back().y, p_last.x, p_last.y);
    if (d > 1e-3) {
      output_path.push_back(p_last);
    } else {
      output_path.back() = p_last;
    }
  }

  return true;
}

bool PostProcessor::resample(const Path& path, Path& resampled_path, std::vector<std::pair<std::string, std::string>>& logs) {
  if (path.size() < 2) {
    resampled_path = path;
    return true;
  }

  resampled_path.clear();
  double step_size = config_.resampler.resolution;
  if (step_size <= 1e-4) {
    step_size = 0.1;
  }

  double total_dist = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    total_dist += dist(path[i].x, path[i].y, path[i-1].x, path[i-1].y);
  }
  resampled_path.reserve(static_cast<size_t>(total_dist / step_size) + path.size());

  Path segment;
  segment.push_back(path[0]);

  for (size_t i = 0; i < path.size() - 1; ++i) {
    const auto& curr = path[i];
    const auto& next = path[i + 1];

    bool direction_changed = (curr.direction != next.direction);

    if (direction_changed) {
      Path segment_resampled;
      resampleSegment(segment, segment_resampled, logs);

      if (resampled_path.empty()) {
        resampled_path = segment_resampled;
      } else {
        if (segment_resampled.size() > 1) {
          resampled_path.insert(resampled_path.end(), segment_resampled.begin() + 1, segment_resampled.end());
        }
      }

      segment.clear();
      PathPoint new_point = curr;
      new_point.direction = next.direction;
      segment.push_back(new_point);
      segment.push_back(next);
    } else {
      segment.push_back(next);
    }
  }

  if (!segment.empty()) {
    Path segment_resampled;
    resampleSegment(segment, segment_resampled, logs);

    if (resampled_path.empty()) {
      resampled_path = segment_resampled;
    } else {
      if (segment_resampled.size() > 1) {
        resampled_path.insert(resampled_path.end(), segment_resampled.begin() + 1, segment_resampled.end());
      }
    }
  }

  double s = 0.0;
  if (!resampled_path.empty()) resampled_path[0].s = 0.0;
  for (size_t i = 1; i < resampled_path.size(); ++i) {
    double d = dist(resampled_path[i].x, resampled_path[i].y, resampled_path[i-1].x, resampled_path[i-1].y);
    s += d;
    resampled_path[i].s = s;
  }

  // Post-processing Resampled Path: Align/blend yaw at direction change cusps and apply safeguards
  for (size_t i = 1; i < resampled_path.size(); ++i) {
    if (resampled_path[i-1].direction != resampled_path[i].direction) {
      // 1. Exact Cusp points (where direction changes) must have zero curvature for safety
      resampled_path[i-1].kappa = 0.0;
      resampled_path[i].kappa = 0.0;

      // 2. Yaw blending over a sliding window
      double theta_e = resampled_path[i-1].theta;
      const int target_dir = resampled_path[i].direction;
      const int N = config_.resampler.yaw_blending_width;
      if (N > 0) {
        double cusp_diff = std::abs(wrap_to_pi(resampled_path[i].theta - theta_e));
        if (cusp_diff < config_.resampler.cusp_angular_threshold_rad) {
          // Blend heading (yaw)
          for (int k = 0; k < N; ++k) {
            if (i + k < resampled_path.size() && resampled_path[i+k].direction == target_dir) {
              double weight = static_cast<double>(k) / N;
              double diff = wrap_to_pi(resampled_path[i+k].theta - theta_e);
              resampled_path[i+k].theta = wrap_to_pi(theta_e + weight * diff);
            }
          }
          // Update curvature (kappa = dtheta / ds) for the blended window with denominator guard
          for (int k = 1; k <= N; ++k) {
            size_t idx = i + k;
            if (idx < resampled_path.size() && resampled_path[idx].direction == target_dir) {
              double ds = dist(resampled_path[idx].x, resampled_path[idx].y, resampled_path[idx-1].x, resampled_path[idx-1].y);
              resampled_path[idx].kappa = (ds > 0.02)
                ? std::clamp(wrap_to_pi(resampled_path[idx].theta - resampled_path[idx-1].theta) / ds, -max_kappa_, max_kappa_)
                : 0.0;
            }
          }
        } else {
          char buf[128];
          std::snprintf(buf, sizeof(buf), "Cusp heading discontinuity too large (%.2f deg), skipping yaw blending.", rad2deg(cusp_diff));
          logs.push_back({"WARN", std::string(buf)});
        }
      }
    }
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
      double ds = dist(path[prev_end].x, path[prev_end].y, path[seg_start].x, path[seg_start].y);
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

  // Apply velocity profile smoothing and recalculate kinematics
  smoothVelocityProfile(path, limits, logs);

  return true;
}

void PostProcessor::smoothVelocityProfile(Path& path, const KinematicLimits& limits, std::vector<std::pair<std::string, std::string>>& logs) {
  if (path.size() < 3) return;

  // 1. Smooth the velocity profile of the entire path using 5-point moving average (3 passes)
  std::vector<double> smoothed_v(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    smoothed_v[i] = path[i].v;
  }

  for (int iter = 0; iter < 3; ++iter) {
    std::vector<double> next_v = smoothed_v;
    if (path.size() > 4) {
      for (size_t i = 2; i < path.size() - 2; ++i) {
        // Keep Cusp velocity at 0 (direction changes or stop points)
        if (path[i].direction != path[i-1].direction || path[i].direction != path[i+1].direction ||
            path[i].direction != path[i-2].direction || path[i].direction != path[i+2].direction ||
            smoothed_v[i] < 1e-3 || smoothed_v[i-1] < 1e-3 || smoothed_v[i-2] < 1e-3 ||
            smoothed_v[i+1] < 1e-3 || smoothed_v[i+2] < 1e-3) {
          next_v[i] = smoothed_v[i];
        } else {
          next_v[i] = (smoothed_v[i-2] + smoothed_v[i-1] + smoothed_v[i] + smoothed_v[i+1] + smoothed_v[i+2]) / 5.0;
        }
      }
    } else {
      for (size_t i = 1; i < path.size() - 1; ++i) {
        if (path[i].direction != path[i-1].direction || path[i].direction != path[i+1].direction ||
            smoothed_v[i] < 1e-3 || smoothed_v[i-1] < 1e-3 || smoothed_v[i+1] < 1e-3) {
          next_v[i] = smoothed_v[i];
        } else {
          next_v[i] = (smoothed_v[i-1] + smoothed_v[i] + smoothed_v[i+1]) / 3.0;
        }
      }
    }
    smoothed_v = std::move(next_v);
  }

  // Copy pre-smoothed velocity to path.v so it can be used as v_static limit
  for (size_t i = 0; i < path.size(); ++i) {
    path[i].v = smoothed_v[i];
  }

  // 2. Re-run Kinematic Profiling per segment with limit_by_input_v = true
  std::vector<std::pair<size_t, size_t>> segments = splitIntoSegments(path);
  double current_time_offset = 0.0;
  double start_vel = path[0].v;

  for (size_t j = 0; j < segments.size(); ++j) {
    size_t seg_start = segments[j].first;
    size_t seg_end = segments[j].second;
    size_t seg_len = seg_end - seg_start + 1;

    // Create a temporary path for this segment
    Path seg_path(path.begin() + seg_start, path.begin() + seg_end + 1);

    // Determine target start and goal velocities for this segment
    double seg_start_v = (j == 0) ? start_vel : 0.0;
    double seg_goal_v = (j == segments.size() - 1) ? config_.profiler.goal_vel : 0.0;

    // Profile the segment, limiting by pre-smoothed velocity
    profileKinematicPass(seg_path, seg_start_v, 0.0, seg_goal_v, 0.0, limits, logs, true);

    // If this is not the first segment, calculate transition time from the end of the previous segment
    if (j > 0) {
      size_t prev_end = segments[j - 1].second;
      double ds = dist(path[prev_end].x, path[prev_end].y, path[seg_start].x, path[seg_start].y);
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

  // 3) Apply 5-point Moving Average Filter on Acceleration multiple times to eliminate remaining Jerk spikes (15 passes)
  if (path.size() > 4) {
    for (int iter = 0; iter < 15; ++iter) {
      std::vector<double> smoothed_acc(path.size());
      smoothed_acc[0] = path[0].a;
      smoothed_acc[1] = path[1].a;
      smoothed_acc[path.size() - 2] = path[path.size() - 2].a;
      smoothed_acc[path.size() - 1] = path.back().a;
      for (size_t i = 2; i < path.size() - 2; ++i) {
        smoothed_acc[i] = (path[i-2].a + path[i-1].a + path[i].a + path[i+1].a + path[i+2].a) / 5.0;
      }
      for (size_t i = 0; i < path.size(); ++i) {
        path[i].a = std::clamp(smoothed_acc[i], -std::abs(limits.max_decel), std::abs(limits.max_accel));
      }
    }
  } else if (path.size() > 2) {
    for (int iter = 0; iter < 15; ++iter) {
      std::vector<double> smoothed_acc(path.size());
      smoothed_acc[0] = path[0].a;
      smoothed_acc[path.size() - 1] = path.back().a;
      for (size_t i = 1; i < path.size() - 1; ++i) {
        smoothed_acc[i] = (path[i-1].a + path[i].a + path[i+1].a) / 3.0;
      }
      for (size_t i = 0; i < path.size(); ++i) {
        path[i].a = std::clamp(smoothed_acc[i], -std::abs(limits.max_decel), std::abs(limits.max_accel));
      }
    }
  }
}

TrajectoryDiagnostic PostProcessor::validateTrajectory(const Path& path, std::vector<std::pair<std::string, std::string>>& logs) const {
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

void PostProcessor::validateVelocity(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double max_vel = config_.profiler.max_vel;
  const double vel_epsilon = 0.01;
  for (size_t i = 0; i < path.size(); ++i) {
    double abs_v = std::abs(path[i].v);
    if (abs_v > max_vel + vel_epsilon) {
      diag.vel_ok = false;
      diag.vel_violations++;
      diag.max_vel_violation = std::max(diag.max_vel_violation, abs_v - max_vel);
    }
  }
  if (!diag.vel_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Velocity check failed: %d points exceeded max velocity limit (max_vel: %.4f). Max violation: %.4f m/s.",
      diag.vel_violations, max_vel, diag.max_vel_violation);
    logs.push_back({"WARN", std::string(buf)});
  }
}

void PostProcessor::validateAcceleration(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double max_accel = std::abs(config_.profiler.max_accel);
  const double max_decel = std::abs(config_.profiler.max_decel);
  const double acc_epsilon = 0.01;
  for (size_t i = 0; i < path.size(); ++i) {
    double a = path[i].a;
    if (a > max_accel + acc_epsilon) {
      diag.acc_ok = false;
      diag.acc_violations++;
      diag.max_acc_violation = std::max(diag.max_acc_violation, a - max_accel);
    } else if (a < -max_decel - acc_epsilon) {
      diag.acc_ok = false;
      diag.acc_violations++;
      diag.max_acc_violation = std::max(diag.max_acc_violation, -a - max_decel);
    }
  }
  if (!diag.acc_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Acceleration check failed: %d points exceeded limits (accel: %.4f, decel: %.4f). Max violation: %.4f m/s^2.",
      diag.acc_violations, max_accel, max_decel, diag.max_acc_violation);
    logs.push_back({"WARN", std::string(buf)});
  }
}

void PostProcessor::validateJerk(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double max_jerk = config_.profiler.max_jerk;
  const double jerk_epsilon = 0.01;
  for (size_t i = 1; i < path.size(); ++i) {
    double dt = path[i].t - path[i-1].t;
    if (dt <= 1e-6) continue;
    if (std::abs(path[i].v) < 0.1 || std::abs(path[i-1].v) < 0.1) continue;

    double jerk_val = std::abs(path[i].a - path[i-1].a) / dt;
    if (jerk_val > max_jerk + jerk_epsilon) {
      diag.jerk_ok = false;
      diag.jerk_violations++;
      diag.max_jerk_violation = std::max(diag.max_jerk_violation, jerk_val - max_jerk);
    }
  }
  if (!diag.jerk_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Jerk check failed: %d points exceeded max jerk limit (max_jerk: %.4f). Max violation: %.4f m/s^3.",
      diag.jerk_violations, max_jerk, diag.max_jerk_violation);
    logs.push_back({"WARN", std::string(buf)});
  }
}

void PostProcessor::validateSteeringVelocity(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double max_steer_vel_limit = 0.6108; // 35.0 deg/s in rad/s
  const double steer_epsilon = 0.01;
  for (size_t i = 1; i < path.size(); ++i) {
    double dt = path[i].t - path[i-1].t;
    if (dt <= 1e-6) continue;
    if (std::abs(path[i].v) < 0.1 || std::abs(path[i-1].v) < 0.1) continue;

    double phi_i = std::atan(wheelbase_ * path[i].kappa);
    double phi_prev = std::atan(wheelbase_ * path[i-1].kappa);
    double steer_vel = std::abs(phi_i - phi_prev) / dt;
    if (steer_vel > max_steer_vel_limit + steer_epsilon) {
      diag.steer_vel_ok = false;
      diag.steer_vel_violations++;
      diag.max_steer_vel_violation = std::max(diag.max_steer_vel_violation, steer_vel - max_steer_vel_limit);
    }
  }
  if (!diag.steer_vel_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Steering Velocity check failed: %d points exceeded limit (max_steer_vel: %.4f rad/s). Max violation: %.4f rad/s.",
      diag.steer_vel_violations, max_steer_vel_limit, diag.max_steer_vel_violation);
    logs.push_back({"WARN", std::string(buf)});
  }
}

void PostProcessor::validateCurvature(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double kappa_epsilon = 1e-3;
  for (size_t i = 0; i < path.size(); ++i) {
    double abs_kappa = std::abs(path[i].kappa);
    if (abs_kappa > max_kappa_ + kappa_epsilon) {
      diag.curvature_ok = false;
      diag.curv_violations++;
      diag.max_curv_violation = std::max(diag.max_curv_violation, abs_kappa - max_kappa_);
    }
  }
  if (!diag.curvature_ok) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "Curvature check failed: %d points exceeded max curvature limit (max_kappa: %.4f). Max violation: %.4f rad/m.",
      diag.curv_violations, max_kappa_, diag.max_curv_violation);
    logs.push_back({"WARN", std::string(buf)});
  }
}

void PostProcessor::validateYawAlignment(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double yaw_tol = 0.087; // ~5 deg
  const double resolution = config_.resampler.resolution > 1e-4 ? config_.resampler.resolution : 0.1;

  // Safe index range to search for a nearby cusp (approx 0.3 meters search radius)
  const int search_window = std::max(3, static_cast<int>(std::round(0.3 / resolution)));

  for (size_t i = 0; i < path.size(); ++i) {
    // Exclude points near direction changes (cusp zones) by verifying the actual physical distance to the Cusp
    bool near_cusp = false;
    for (int w = -search_window; w <= search_window; ++w) {
      int idx = static_cast<int>(i) + w;
      if (idx >= 0 && idx < static_cast<int>(path.size())) {
        if (path[idx].direction != path[i].direction) {
          // Found a cusp. Verify if it is within the 0.2m physical limit.
          double actual_dist = dist(path[i].x, path[i].y, path[idx].x, path[idx].y);
          if (actual_dist < 0.2) {
            near_cusp = true;
            break;
          }
        }
      }
    }
    if (near_cusp) continue;

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

void PostProcessor::validateTimestamps(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const {
  const double time_tol = 1e-3; // 1 ms

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
      double ds = dist(path[i-1].x, path[i-1].y, path[i].x, path[i].y);
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

} // namespace carmaker_planning
