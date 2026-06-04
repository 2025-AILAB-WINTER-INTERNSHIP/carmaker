/**
 * @file post_processing.h
 * @brief Consolidated header for post-processing pipeline steps (smoothing, resampling, velocity profiling).
 */
#ifndef CARMAKER_PLANNING_POST_PROCESSING_H
#define CARMAKER_PLANNING_POST_PROCESSING_H

#include "carmaker_planning/types.h"
#include "carmaker_planning/global_map.h"

namespace carmaker_planning {

struct KinematicLimits {
  double max_vel;
  double max_accel;
  double max_decel;
  double max_jerk;
  double max_steer_vel;
  double max_lat_acc;
  double min_vel_denom;
};

class PostProcessor {
public:
  explicit PostProcessor(const GlobalMainConfig& config);
  ~PostProcessor() = default;
  PostProcessor(const PostProcessor&) = delete;
  PostProcessor& operator=(const PostProcessor&) = delete;

  bool process(GlobalPlanningResult& result,
               const GlobalMap& map,
               double start_vel,
               bool enable_smoothing,
               bool enable_resampling,
               bool enable_profiling);

private:
  bool smooth(Path& path, const GlobalMap& map, std::vector<std::pair<std::string, std::string>>& logs);
  bool resample(const Path& path, Path& resampled_path, std::vector<std::pair<std::string, std::string>>& logs);
  bool resampleSegment(const Path& input_segment, Path& output_path, std::vector<std::pair<std::string, std::string>>& logs);
  bool profile(Path& path, double start_vel, std::vector<std::pair<std::string, std::string>>& logs);
  void profileKinematicPass(Path& path, double start_v, double start_a,
                            double goal_v, double goal_a, const KinematicLimits& limits,
                            std::vector<std::pair<std::string, std::string>>& logs,
                            bool limit_by_input_v = false);
  std::vector<std::pair<size_t, size_t>> splitIntoSegments(const Path& path) const;
  void updateGeometryProperties(Path& path) const;

  GlobalPostProcessConfig config_;
  double max_kappa_ = 0.2;
  double wheelbase_ = 2.97;
};

class TrajectoryValidator {
public:
  TrajectoryValidator(const GlobalPostProcessConfig& config, double max_kappa, double wheelbase);
  ~TrajectoryValidator() = default;
  TrajectoryValidator(const TrajectoryValidator&) = delete;
  TrajectoryValidator& operator=(const TrajectoryValidator&) = delete;

  TrajectoryDiagnostic validate(const Path& path, std::vector<std::pair<std::string, std::string>>& logs) const;

private:
  void validateCurvature(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const;
  void validateYawAlignment(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const;
  void validateTimestamps(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const;
  void validateVelocity(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const;
  void validateAcceleration(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const;
  void validateJerk(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const;
  void validateSteeringVelocity(const Path& path, TrajectoryDiagnostic& diag, std::vector<std::pair<std::string, std::string>>& logs) const;
  bool isNearCusp(const Path& path, size_t index, double radius_threshold = 0.2) const;

  GlobalPostProcessConfig config_;
  double max_kappa_;
  double wheelbase_;
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_POST_PROCESSING_H
