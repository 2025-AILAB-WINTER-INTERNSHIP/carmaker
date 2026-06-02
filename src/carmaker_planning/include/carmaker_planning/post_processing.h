/**
 * @file post_processing.h
 * @brief Consolidated header for post-processing pipeline steps (smoothing, resampling, velocity profiling).
 */
#ifndef CARMAKER_PLANNING_POST_PROCESSING_H
#define CARMAKER_PLANNING_POST_PROCESSING_H

#include "carmaker_planning/types.h"
#include "carmaker_planning/global_map.h"

namespace carmaker_planning {

class CubicSpline {
public:
  CubicSpline() = default;
  ~CubicSpline() = default;

  bool fit(const std::vector<double>& s, const std::vector<double>& f) {
    const size_t n = s.size();
    if (n < 3) {
      clear();
      return false;
    }

    std::vector<double> h(n - 1);
    for (size_t i = 0; i < n - 1; ++i) {
      h[i] = s[i+1] - s[i];
      if (h[i] <= 1e-6) {
        clear();
        return false;
      }
    }

    std::vector<double> c_prime(n, 0.0);
    std::vector<double> M(n, 0.0);

    double b1 = 2.0 * (h[0] + h[1]);
    if (std::abs(b1) < 1e-9) {
      clear();
      return false;
    }
    c_prime[1] = h[1] / b1;
    M[1] = 6.0 * ((f[2] - f[1]) / h[1] - (f[1] - f[0]) / h[0]) / b1;

    for (size_t i = 2; i < n - 1; ++i) {
      double denom = 2.0 * (h[i-1] + h[i]) - h[i-1] * c_prime[i-1];
      if (std::abs(denom) < 1e-9) {
        clear();
        return false;
      }
      c_prime[i] = h[i] / denom;
      double d_i = 6.0 * ((f[i+1] - f[i]) / h[i] - (f[i] - f[i-1]) / h[i-1]);
      M[i] = (d_i - h[i-1] * M[i-1]) / denom;
    }

    M[n-1] = 0.0;
    M[0] = 0.0;
    for (int i = static_cast<int>(n) - 3; i >= 1; --i) {
      M[i] = M[i] - c_prime[i] * M[i+1];
    }

    n_ = n;
    s_ = s;
    f_ = f;
    M_ = std::move(M);
    return true;
  }

  void clear() {
    n_ = 0;
    s_.clear();
    f_.clear();
    M_.clear();
  }

  void evaluate(double s, double& val, double& deriv, double& second_deriv) const {
    if (n_ == 0) {
      val = 0.0; deriv = 0.0; second_deriv = 0.0;
      return;
    }
    if (n_ == 1) {
      val = f_[0]; deriv = 0.0; second_deriv = 0.0;
      return;
    }

    size_t idx = 0;
    if (s <= s_[0]) {
      idx = 0;
    } else if (s >= s_[n_-1]) {
      idx = n_ - 2;
    } else {
      auto it = std::lower_bound(s_.begin(), s_.end(), s);
      idx = std::distance(s_.begin(), it) - 1;
      if (idx >= n_ - 1) idx = n_ - 2;
    }

    double h = s_[idx+1] - s_[idx];
    double s_diff_curr = s - s_[idx];
    double s_diff_next = s_[idx+1] - s;

    val = (M_[idx] / (6.0 * h)) * std::pow(s_diff_next, 3) +
          (M_[idx+1] / (6.0 * h)) * std::pow(s_diff_curr, 3) +
          (f_[idx] / h - h * M_[idx] / 6.0) * s_diff_next +
          (f_[idx+1] / h - h * M_[idx+1] / 6.0) * s_diff_curr;

    deriv = -(M_[idx] / (2.0 * h)) * std::pow(s_diff_next, 2) +
            (M_[idx+1] / (2.0 * h)) * std::pow(s_diff_curr, 2) -
            (f_[idx] / h) + (h * M_[idx] / 6.0) +
            (f_[idx+1] / h) - (h * M_[idx+1] / 6.0);

    second_deriv = (M_[idx] / h) * s_diff_next + (M_[idx+1] / h) * s_diff_curr;
  }

private:
  size_t n_ = 0;
  std::vector<double> s_;
  std::vector<double> f_;
  std::vector<double> M_;
};

class PathSmoother {
public:
  explicit PathSmoother(const GlobalMainConfig& config);
  ~PathSmoother() = default;
  PathSmoother(const PathSmoother&) = delete;
  PathSmoother& operator=(const PathSmoother&) = delete;

  bool smooth(Path& path, const GlobalMap& map);

private:
  GlobalPostProcessConfig config_;
};

class PathResampler {
public:
  explicit PathResampler(const GlobalMainConfig& config);
  ~PathResampler() = default;
  PathResampler(const PathResampler&) = delete;
  PathResampler& operator=(const PathResampler&) = delete;

  bool resample(const Path& path, Path& resampled_path);

private:
  bool resampleSegment(const Path& path, Path& resampled_path);
  GlobalPostProcessConfig config_;
  CubicSpline spline_x_;
  CubicSpline spline_y_;
};

struct KinematicLimits {
  double max_vel;
  double max_accel;
  double max_decel;
  double max_jerk;
  double max_lat_acc;
};

class VelocityProfiler {
public:
  explicit VelocityProfiler(const GlobalMainConfig& config);
  ~VelocityProfiler() = default;
  VelocityProfiler(const VelocityProfiler&) = delete;
  VelocityProfiler& operator=(const VelocityProfiler&) = delete;

  bool profile(Path& path, double start_vel);

private:
  GlobalPostProcessConfig config_;
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_POST_PROCESSING_H
