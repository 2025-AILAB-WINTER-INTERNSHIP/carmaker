/**
 * @file trajectory_state_machine.cpp
 * @brief Active segment state machine implementation.
 */

#include "carmaker_planning/trajectory_state_machine.h"
#include "carmaker_planning/math.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace carmaker_planning {

TrajectoryStateMachine::TrajectoryStateMachine() : TrajectoryStateMachine(Config()) {}

TrajectoryStateMachine::TrajectoryStateMachine(const Config& config) {
  configure(config);
}

void TrajectoryStateMachine::configure(const Config& config) {
  config_ = config;
  config_.endpoint_xy_tol = std::max(0.0, config_.endpoint_xy_tol);
  config_.endpoint_yaw_tol = std::max(0.0, config_.endpoint_yaw_tol);
  config_.stop_vel_tol = std::max(0.0, config_.stop_vel_tol);
  config_.precision_zone_distance = std::max(0.0, config_.precision_zone_distance);
  config_.stop_duration = std::max(0.0, config_.stop_duration);
  config_.presteer_duration = std::max(0.0, config_.presteer_duration);
  config_.idle_after_finish_duration = std::max(0.0, config_.idle_after_finish_duration);
}

void TrajectoryStateMachine::setGlobalPath(const Path& path) {
  segment_manager_.setGlobalPath(path);
  state_ = segment_manager_.hasPath() ? PlannerState::kTracking : PlannerState::kIdle;
  transition_until_sec_ = 0.0;
}

TrajectoryStateMachine::TrajectoryDecision
TrajectoryStateMachine::update(const carmaker_planning::State& ego, double now_sec) {
  if (!segment_manager_.hasPath() || segment_manager_.isFinished() ||
      state_ == PlannerState::kIdle) {
    state_ = PlannerState::kIdle;
    return makeDecision(TrajectorySource::kNone,
                        TrajectoryIntent::kNone,
                        segment_manager_.isFinished());
  }

  PathPoint endpoint;
  if (!activeEndpoint(ego, endpoint)) {
    state_ = PlannerState::kIdle;
    return makeDecision(TrajectorySource::kNone, TrajectoryIntent::kNone, true);
  }

  if (state_ == PlannerState::kTracking && shouldStartStopping(ego, endpoint)) {
    state_ = PlannerState::kStopping;
    transition_until_sec_ = now_sec + config_.stop_duration;
  }

  if (state_ == PlannerState::kStopping) {
    const bool hold_done = now_sec >= transition_until_sec_;
    const bool precise_reached =
        endpointReachedWithTolerances(ego,
                                      endpoint,
                                      config_.endpoint_xy_tol,
                                      config_.endpoint_yaw_tol,
                                      config_.stop_vel_tol);
    if (hold_done && precise_reached) {
      if (activeSegmentIsFinal()) {
        state_ = PlannerState::kFinishedHold;
        transition_until_sec_ = now_sec + config_.idle_after_finish_duration;
        return makeDecision(TrajectorySource::kActiveSegment,
                            TrajectoryIntent::kFinishedHold,
                            true);
      }

      segment_manager_.advanceToNextSegment();
      state_ = PlannerState::kPresteering;
      transition_until_sec_ = now_sec + config_.presteer_duration;
      return makeDecision(TrajectorySource::kActiveSegment,
                          TrajectoryIntent::kPresteerNext);
    }
    return makeDecision(TrajectorySource::kActiveSegment,
                        TrajectoryIntent::kStopCurrent);
  }

  if (state_ == PlannerState::kPresteering) {
    if (now_sec < transition_until_sec_) {
      return makeDecision(TrajectorySource::kActiveSegment,
                          TrajectoryIntent::kPresteerNext);
    }
    state_ = PlannerState::kTracking;
  }

  if (state_ == PlannerState::kFinishedHold) {
    if (now_sec >= transition_until_sec_) {
      segment_manager_.advanceToNextSegment();
      state_ = PlannerState::kIdle;
      return makeDecision(TrajectorySource::kNone, TrajectoryIntent::kNone, true);
    }
    return makeDecision(TrajectorySource::kActiveSegment,
                        TrajectoryIntent::kFinishedHold, true);
  }

  const bool use_final_approach_speed_cap = inFinalApproachZone(ego, endpoint);
  if (config_.mode == Mode::kGlobal) {
    return makeDecision(TrajectorySource::kActiveSegment,
                        TrajectoryIntent::kTrack,
                        false,
                        use_final_approach_speed_cap);
  }
  return makeDecision(TrajectorySource::kLocalToEndpoint,
                      TrajectoryIntent::kTrack,
                      false,
                      use_final_approach_speed_cap);
}

TrajectoryStateMachine::TrajectoryDecision TrajectoryStateMachine::stopForTimeout(double) {
  if (!segment_manager_.hasPath() || segment_manager_.isFinished()) {
    return makeDecision(TrajectorySource::kNone,
                        TrajectoryIntent::kEmergencyStop,
                        segment_manager_.isFinished());
  }
  return makeDecision(TrajectorySource::kActiveSegment,
                      TrajectoryIntent::kEmergencyStop);
}

TrajectoryStateMachine::Mode TrajectoryStateMachine::parseMode(const std::string& mode) {
  return mode == "global" ? Mode::kGlobal : Mode::kLocal;
}

bool TrajectoryStateMachine::activeEndpoint(const carmaker_planning::State&,
                                             PathPoint& endpoint) const {
  const PathPoint* ep = segment_manager_.activeEndpoint();
  if (!ep) return false;
  endpoint = *ep;
  return true;
}

bool TrajectoryStateMachine::shouldStartStopping(const carmaker_planning::State& ego,
                                                  const PathPoint& endpoint) const {
  return endpointPoseReached(ego, endpoint, config_.endpoint_xy_tol, config_.endpoint_yaw_tol);
}

bool TrajectoryStateMachine::endpointPoseReached(
    const carmaker_planning::State& ego,
    const PathPoint& endpoint,
    double xy_tol,
    double yaw_tol) const {
  const double d_xy = dist(ego.x, ego.y, endpoint.x, endpoint.y);
  const double d_yaw = std::abs(wrap_to_pi(ego.theta - endpoint.theta));
  return d_xy <= xy_tol && d_yaw <= yaw_tol;
}

bool TrajectoryStateMachine::endpointReachedWithTolerances(
    const carmaker_planning::State& ego,
    const PathPoint& endpoint,
    double xy_tol,
    double yaw_tol,
    double vel_tol) const {
  const double v = std::abs(ego.v);
  return endpointPoseReached(ego, endpoint, xy_tol, yaw_tol) && v <= vel_tol;
}

bool TrajectoryStateMachine::inFinalApproachZone(const carmaker_planning::State& ego,
                                       const PathPoint& endpoint) const {
  if (!activeSegmentIsFinal()) return false;
  const double d_xy = dist(ego.x, ego.y, endpoint.x, endpoint.y);
  return d_xy <= config_.precision_zone_distance &&
         !endpointPoseReached(ego, endpoint, config_.endpoint_xy_tol, config_.endpoint_yaw_tol);
}

bool TrajectoryStateMachine::activeSegmentIsFinal() const {
  return segment_manager_.segmentCount() > 0 &&
         segment_manager_.activeSegmentIndex() + 1 >= segment_manager_.segmentCount();
}

TrajectoryStateMachine::TrajectoryDecision
TrajectoryStateMachine::makeDecision(TrajectorySource path_source,
                           TrajectoryIntent intent,
                           bool finished,
                           bool use_final_approach_speed_cap) const {
  Path active_segment;
  Path global_path;
  PathPoint endpoint;
  const bool has_endpoint = activeEndpoint(State{}, endpoint);
  if (path_source == TrajectorySource::kActiveSegment ||
      path_source == TrajectorySource::kLocalToEndpoint) {
    active_segment = segment_manager_.activeSegmentPath();
  }
  if (path_source == TrajectorySource::kLocalToEndpoint) {
    global_path = segment_manager_.globalPath();
  }

  TrajectoryDecision result;
  result.publish = path_source != TrajectorySource::kNone && !active_segment.empty();
  result.finished = finished;
  result.use_final_approach_speed_cap = use_final_approach_speed_cap;
  result.active_direction = pathDirection(active_segment, has_endpoint ? endpoint.direction : 1);
  result.active_segment_index = segment_manager_.activeSegmentIndex();
  result.state = state_;
  result.intent = intent;
  result.path_source = path_source;
  result.active_segment = std::move(active_segment);
  result.global_path = std::move(global_path);
  result.endpoint = endpoint;
  return result;
}

int TrajectoryStateMachine::pathDirection(const Path& path, int fallback) {
  for (const auto& pt : path) {
    if (pt.direction < 0) return -1;
    if (pt.direction > 0) return 1;
  }
  return fallback < 0 ? -1 : 1;
}

} // namespace carmaker_planning
