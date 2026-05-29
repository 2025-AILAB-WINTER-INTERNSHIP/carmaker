/**
 * @file post_processing.cpp
 * @brief Consolidated implementation for post-processing pipeline steps.
 */

#include "carmaker_planning/post_processing.h"
#include "carmaker_planning/math.h"
#include <cmath>
#include <algorithm>

namespace carmaker_planning {

namespace {

void profileKinematicPass(Path& path, double start_v, double start_a,
                          double goal_v, double goal_a, const KinematicLimits& limits) {
  if (path.empty()) return;
  const int n = static_cast<int>(path.size());
  const double max_acc = std::abs(limits.max_acc);
  const double max_decel = -std::abs(limits.max_decel);
  const double max_jerk = std::abs(limits.max_jerk);

  std::vector<double> v_static(n);
  for (int i = 0; i < n; ++i) {
    double v_curve = (std::abs(path[i].kappa) > 1e-3)
      ? std::sqrt(limits.max_lat_acc / std::abs(path[i].kappa)) : limits.max_vel;
    v_static[i] = std::min(limits.max_vel, v_curve);
  }

  std::vector<double> v_fwd(n), a_fwd(n);
  v_fwd[0] = std::max(0.0, start_v);
  a_fwd[0] = std::clamp(start_a, max_decel, max_acc);
  for (int i = 1; i < n; ++i) {
    const double ds = dist(path[i-1].x, path[i-1].y, path[i].x, path[i].y);
    const double dt = (ds > 1e-6) ? ds / std::max(0.1, v_fwd[i-1]) : 0.1;
    double a_lim = std::min(max_acc, a_fwd[i-1] + max_jerk * dt);
    v_fwd[i] = std::sqrt(std::max(0.0, v_fwd[i-1]*v_fwd[i-1] + 2.0*a_lim*ds));
    a_fwd[i] = std::clamp((ds > 1e-6) ? (v_fwd[i]*v_fwd[i]-v_fwd[i-1]*v_fwd[i-1])/(2.0*ds)
                          : a_fwd[i-1], max_decel, max_acc);
  }

  std::vector<double> v_bwd(n), a_bwd(n);
  v_bwd[n-1] = std::max(0.0, goal_v);
  a_bwd[n-1] = std::clamp(goal_a, max_decel, max_acc);
  for (int i = n-2; i >= 0; --i) {
    const double ds = dist(path[i].x, path[i].y, path[i+1].x, path[i+1].y);
    const double dt = (ds > 1e-6) ? ds / std::max(0.1, v_bwd[i+1]) : 0.1;
    double a_lim = std::max(max_decel, a_bwd[i+1] - max_jerk * dt);
    v_bwd[i] = std::sqrt(std::max(0.0, v_bwd[i+1]*v_bwd[i+1] - 2.0*a_lim*ds));
    a_bwd[i] = std::clamp((ds > 1e-6) ? (v_bwd[i+1]*v_bwd[i+1]-v_bwd[i]*v_bwd[i])/(2.0*ds)
                          : a_bwd[i+1], max_decel, max_acc);
  }

  for (int i = 0; i < n; ++i) {
    path[i].v = std::min({v_fwd[i], v_bwd[i], v_static[i]});
    if (i == 0) {
      path[i].a = a_fwd[0];
      path[i].t = 0.0;
    } else {
      const double ds = dist(path[i-1].x, path[i-1].y, path[i].x, path[i].y);
      const double dt = (ds > 1e-6) ? ds / std::max(0.1, (path[i].v + path[i-1].v) / 2.0) : 0.1;
      const double acc_dt = std::max(0.01, dt);
      path[i].a = std::clamp((path[i].v - path[i-1].v) / acc_dt, max_decel, max_acc);
      path[i].t = path[i-1].t + dt;
    }
  }
}

std::vector<std::pair<size_t, size_t>> splitIntoSegments(const Path& path) {
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

} // namespace

// ── PathSmoother Implementation ──────────────────────────────────────

PathSmoother::PathSmoother(const GlobalMainConfig& config)
: config_(config.post_process) {}

bool PathSmoother::smooth(Path& path, const GlobalMap& map) {
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

        // Laplace smoothing within the segment
        correction_x += w_smooth * (path[global_idx + 1].x + path[global_idx - 1].x - 2.0 * x);
        correction_y += w_smooth * (path[global_idx + 1].y + path[global_idx - 1].y - 2.0 * y);

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
  }

  // 3. Re-calculate yaw and curvature for the entire path using segment boundaries
  for (const auto& seg : segments) {
    size_t seg_start = seg.first;
    size_t seg_end = seg.second;
    int seg_len = static_cast<int>(seg_end - seg_start + 1);

    for (int i = 0; i < seg_len; ++i) {
      size_t global_idx = seg_start + i;
      double dx = 0.0, dy = 0.0;
      if (seg_len < 2) continue;

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
        path[global_idx].kappa = mengerCurvature(
          path[global_idx - 1].x, path[global_idx - 1].y,
          path[global_idx].x,     path[global_idx].y,
          path[global_idx + 1].x, path[global_idx + 1].y);
      }
    }

    if (seg_len >= 2) {
      path[seg_start].kappa = path[seg_start + 1].kappa;
      path[seg_end].kappa = path[seg_end - 1].kappa;
    }
  }

  return true;
}

// ── PathResampler Implementation ─────────────────────────────────────

PathResampler::PathResampler(const GlobalMainConfig& config)
: config_(config.post_process) {}

bool PathResampler::resampleSegment(const Path& input_segment, Path& output_path) {
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

  const double step_size = config_.resampler.resolution;
  const int num_steps = std::floor(segment_len / step_size);

  size_t current_idx = 0;
  for (int i = 0; i <= num_steps; ++i) {
    double s_req = i * step_size;

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

    PathPoint p;
    p.x = p0.x + ratio * (p1.x - p0.x);
    p.y = p0.y + ratio * (p1.y - p0.y);

    double d_theta = wrap_to_pi(p1.theta - p0.theta);
    p.theta = wrap_to_pi(p0.theta + ratio * d_theta);
    p.direction = p0.direction;
    p.s = 0.0;

    output_path.push_back(p);
  }

  const auto& last_pt = input_segment.back();
  if (output_path.empty()) {
    output_path.push_back(last_pt);
  } else {
    double d = dist(output_path.back().x, output_path.back().y, last_pt.x, last_pt.y);
    if (d > 1e-3) {
      output_path.push_back(last_pt);
    } else {
      output_path.back() = last_pt;
    }
  }

  return true;
}

bool PathResampler::resample(const Path& path, Path& resampled_path) {
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
      resampleSegment(segment, segment_resampled);

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
    resampleSegment(segment, segment_resampled);

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

  // Recalculate kappa (curvature) using Menger curvature for the resampled path.
  int n_res = resampled_path.size();
  if (n_res < 3) {
    for (auto& pt : resampled_path) {
      pt.kappa = 0.0;
    }
  } else {
    for (int i = 1; i < n_res - 1; ++i) {
      if (resampled_path[i-1].direction != resampled_path[i].direction || 
          resampled_path[i+1].direction != resampled_path[i].direction) {
        resampled_path[i].kappa = 0.0;
      } else {
        resampled_path[i].kappa = mengerCurvature(
          resampled_path[i - 1].x, resampled_path[i - 1].y,
          resampled_path[i].x,     resampled_path[i].y,
          resampled_path[i + 1].x, resampled_path[i + 1].y);
      }
    }
    // Handle boundaries/cusps
    resampled_path[0].kappa = (resampled_path[0].direction == resampled_path[1].direction) ? resampled_path[1].kappa : 0.0;
    resampled_path[n_res - 1].kappa = (resampled_path[n_res - 1].direction == resampled_path[n_res - 2].direction) ? resampled_path[n_res - 2].kappa : 0.0;
    
    // Fill in endpoints of segments that were adjacent to direction changes
    for (int i = 1; i < n_res - 1; ++i) {
      if (resampled_path[i-1].direction != resampled_path[i].direction) {
        resampled_path[i].kappa = resampled_path[i+1].kappa;
      } else if (resampled_path[i+1].direction != resampled_path[i].direction) {
        resampled_path[i].kappa = resampled_path[i-1].kappa;
      }
    }
  }

  return true;
}

// ── VelocityProfiler Implementation ──────────────────────────────────

VelocityProfiler::VelocityProfiler(const GlobalMainConfig& config)
: config_(config.post_process) {}

bool VelocityProfiler::profile(Path& path, double start_vel) {
  if (path.empty()) {
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
  limits.max_acc = config_.profiler.max_acc;
  limits.max_decel = config_.profiler.max_dec;
  limits.max_jerk = config_.profiler.max_jerk;
  limits.max_lat_acc = config_.profiler.max_lat_acc;

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
    profileKinematicPass(seg_path, seg_start_v, 0.0, seg_goal_v, 0.0, limits);

    // If this is not the first segment, calculate transition time from the end of the previous segment
    if (j > 0) {
      size_t prev_end = segments[j - 1].second;
      double ds = dist(path[prev_end].x, path[prev_end].y, path[seg_start].x, path[seg_start].y);
      // Since velocity is 0 at both sides of the cusp, we use a default speed (e.g. 0.1) for transition
      double dt = (ds > 1e-6) ? ds / 0.1 : 0.1;
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

} // namespace carmaker_planning
