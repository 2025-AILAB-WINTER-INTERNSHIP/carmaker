/**
 * @file post_processing.h
 * @brief Consolidated header for post-processing pipeline steps (smoothing, resampling, velocity profiling).
 */
#ifndef CARMAKER_PLANNING_POST_PROCESSING_H
#define CARMAKER_PLANNING_POST_PROCESSING_H

#include "carmaker_planning/types.h"
#include "carmaker_planning/global_map.h"

namespace carmaker_planning {

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
};

struct KinematicLimits {
  double max_vel;
  double max_acc;
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
