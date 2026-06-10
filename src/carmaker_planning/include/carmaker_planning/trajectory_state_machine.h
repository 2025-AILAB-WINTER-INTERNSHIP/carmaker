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
    kPresteering
  };

  enum class TrajectoryIntent {
    kNone,
    kTrack,
    kStopCurrent,
    kPresteerNext,
    kEmergencyStop
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
    double segment_transition_xy_tol = 0.3;
    double segment_transition_yaw_tol = 0.2;
    double stop_vel_tol = 0.05;
    double stop_duration = 0.5;
    double presteer_duration = 0.5;
  };

  struct TrajectoryDecision {
    bool publish = false;
    bool finished = false;
    int active_direction = 1;
    size_t active_segment_index = 0;
    PlannerState state = PlannerState::kIdle;
    TrajectoryIntent intent = TrajectoryIntent::kNone;
    TrajectorySource path_source = TrajectorySource::kNone;
    Path active_segment;
    Path global_path;
    PathPoint endpoint;
  };

  struct ArrivalTolerance {
    double xy = 0.0;
    double yaw = 0.0;
    double velocity = 0.0;
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
  size_t activeSegmentIndex() const { return segment_manager_.activeSegmentIndex(); }
  PlannerState state() const { return state_; }

  static Mode parseMode(const std::string& mode);

private:
  bool activeEndpoint(const carmaker_planning::State& ego, PathPoint& endpoint) const;
  bool shouldStartStopping(const carmaker_planning::State& ego,
                           const PathPoint& endpoint) const;
  ArrivalTolerance activeArrivalTolerance() const;
  static bool endpointPoseReached(const carmaker_planning::State& ego,
                                  const PathPoint& endpoint,
                                  ArrivalTolerance tolerance);
  static bool endpointReachedWithTolerances(const carmaker_planning::State& ego,
                                            const PathPoint& endpoint,
                                            ArrivalTolerance tolerance);
  bool activeSegmentIsFinal() const;
  TrajectoryDecision makeDecision(TrajectorySource path_source,
                                  TrajectoryIntent intent,
                                  bool finished = false) const;
  static int pathDirection(const Path& path, int fallback = 1);

  Config config_;
  SegmentManager segment_manager_;
  PlannerState state_ = PlannerState::kIdle;
  double transition_until_sec_ = 0.0;
};

inline const char* trajectoryStateString(TrajectoryStateMachine::PlannerState state) {
  switch (state) {
    case TrajectoryStateMachine::PlannerState::kIdle: return "Idle";
    case TrajectoryStateMachine::PlannerState::kTracking: return "Tracking";
    case TrajectoryStateMachine::PlannerState::kStopping: return "Stopping";
    case TrajectoryStateMachine::PlannerState::kPresteering: return "Presteering";
  }
  return "Unknown";
}

inline const char* trajectoryIntentString(TrajectoryStateMachine::TrajectoryIntent intent) {
  switch (intent) {
    case TrajectoryStateMachine::TrajectoryIntent::kNone: return "None";
    case TrajectoryStateMachine::TrajectoryIntent::kTrack: return "Track";
    case TrajectoryStateMachine::TrajectoryIntent::kStopCurrent: return "StopCurrent";
    case TrajectoryStateMachine::TrajectoryIntent::kPresteerNext: return "PresteerNext";
    case TrajectoryStateMachine::TrajectoryIntent::kEmergencyStop: return "EmergencyStop";
  }
  return "Unknown";
}

inline const char* trajectorySourceString(TrajectoryStateMachine::TrajectorySource source) {
  switch (source) {
    case TrajectoryStateMachine::TrajectorySource::kNone: return "None";
    case TrajectoryStateMachine::TrajectorySource::kActiveSegment: return "ActiveSegment";
    case TrajectoryStateMachine::TrajectorySource::kLocalToEndpoint: return "LocalToEndpoint";
  }
  return "Unknown";
}

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_TRAJECTORY_STATE_MACHINE_H
