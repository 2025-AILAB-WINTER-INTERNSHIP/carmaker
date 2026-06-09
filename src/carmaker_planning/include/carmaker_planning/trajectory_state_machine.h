/**
 * @file trajectory_state_machine.h
 * @brief Active segment state machine and trajectory publish decision model.
 */
#ifndef CARMAKER_PLANNING_TRAJECTORY_STATE_MACHINE_H
#define CARMAKER_PLANNING_TRAJECTORY_STATE_MACHINE_H

#include <cstddef>
#include <string>
#include "carmaker_planning/local_planner.h"

namespace carmaker_planning {

class TrajectoryStateMachine {
public:
  enum class Mode {
    kLocal,
    kGlobal
  };

  enum class PlannerState {
    kIdle,
    kTracking,
    kStopping,
    kPresteering,
    kFinishedHold
  };

  enum class TrajectoryIntent {
    kNone,
    kTrack,
    kStopCurrent,
    kPresteerNext,
    kEmergencyStop,
    kFinishedHold
  };

  enum class TrajectorySource {
    kNone,
    kActiveSegment,
    kLocalToEndpoint
  };

  struct Config {
    Mode mode = Mode::kLocal;
    double endpoint_xy_tol = 0.3;
    double endpoint_yaw_tol = 0.2;
    double stop_vel_tol = 0.05;
    double precision_zone_distance = 0.3;
    double stop_duration = 0.5;
    double presteer_duration = 0.5;
    double idle_after_finish_duration = 0.5;
  };

  struct TrajectoryDecision {
    bool publish = false;
    bool finished = false;
    bool use_final_approach_speed_cap = false;
    int active_direction = 1;
    size_t active_segment_index = 0;
    PlannerState state = PlannerState::kIdle;
    TrajectoryIntent intent = TrajectoryIntent::kNone;
    TrajectorySource path_source = TrajectorySource::kNone;
    Path active_segment;
    Path global_path;
    PathPoint endpoint;
  };

  TrajectoryStateMachine();
  explicit TrajectoryStateMachine(const Config& config);

  void configure(const Config& config);
  void setGlobalPath(const Path& path);
  TrajectoryDecision update(const carmaker_planning::State& ego, double now_sec);
  TrajectoryDecision stopForTimeout(double now_sec);

  const Path& globalPath() const { return segment_manager_.globalPath(); }
  bool hasPath() const { return segment_manager_.hasPath(); }
  bool isIdle() const { return state_ == PlannerState::kIdle; }
  size_t segmentCount() const { return segment_manager_.segmentCount(); }
  PlannerState state() const { return state_; }

  static Mode parseMode(const std::string& mode);

private:
  bool activeEndpoint(const carmaker_planning::State& ego, PathPoint& endpoint) const;
  bool shouldStartStopping(const carmaker_planning::State& ego,
                           const PathPoint& endpoint) const;
  bool endpointPoseReached(const carmaker_planning::State& ego,
                           const PathPoint& endpoint,
                           double xy_tol,
                           double yaw_tol) const;
  bool endpointReachedWithTolerances(const carmaker_planning::State& ego,
                                     const PathPoint& endpoint,
                                     double xy_tol,
                                     double yaw_tol,
                                     double vel_tol) const;
  bool inFinalApproachZone(const carmaker_planning::State& ego,
                           const PathPoint& endpoint) const;
  bool activeSegmentIsFinal() const;
  TrajectoryDecision makeDecision(TrajectorySource path_source,
                                  TrajectoryIntent intent,
                                  bool finished = false,
                                  bool use_final_approach_speed_cap = false) const;
  static int pathDirection(const Path& path, int fallback = 1);

  Config config_;
  SegmentManager segment_manager_;
  PlannerState state_ = PlannerState::kIdle;
  double transition_until_sec_ = 0.0;
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_TRAJECTORY_STATE_MACHINE_H
