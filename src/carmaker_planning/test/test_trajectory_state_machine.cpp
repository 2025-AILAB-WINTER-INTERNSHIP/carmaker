#include <gtest/gtest.h>

#include "carmaker_planning/trajectory_state_machine.h"

namespace carmaker_planning {
namespace {

PathPoint makePoint(double x, int direction) {
  PathPoint pt;
  pt.x = x;
  pt.y = 0.0;
  pt.theta = 0.0;
  pt.direction = direction;
  pt.v = 0.3;
  return pt;
}

Path makeSingleSegmentPath() {
  return {makePoint(0.0, 1), makePoint(1.0, 1)};
}

Path makeTwoSegmentPath() {
  return {makePoint(0.0, 1), makePoint(1.0, 1),
          makePoint(1.0, -1), makePoint(0.0, -1)};
}

State makeEgo(double x, double v = 0.0) {
  State ego;
  ego.x = x;
  ego.y = 0.0;
  ego.theta = 0.0;
  ego.v = v;
  return ego;
}

TrajectoryStateMachine::Config makeConfig(
    TrajectoryStateMachine::Mode mode = TrajectoryStateMachine::Mode::kLocal) {
  TrajectoryStateMachine::Config cfg;
  cfg.mode = mode;
  cfg.endpoint_xy_tol = 0.05;
  cfg.endpoint_yaw_tol = 0.2;
  cfg.stop_vel_tol = 0.05;
  cfg.stop_duration = 0.5;
  cfg.presteer_duration = 0.5;
  cfg.idle_after_finish_duration = 0.5;
  return cfg;
}

TEST(TrajectoryStateMachineTest, OutsideEndpointToleranceKeepsTracking) {
  TrajectoryStateMachine state_machine(makeConfig());
  state_machine.setGlobalPath(makeSingleSegmentPath());

  const auto result = state_machine.update(makeEgo(0.8), 0.0);

  EXPECT_TRUE(result.publish);
  EXPECT_FALSE(result.finished);
  EXPECT_EQ(result.state, TrajectoryStateMachine::PlannerState::kTracking);
  EXPECT_EQ(result.intent, TrajectoryStateMachine::TrajectoryIntent::kTrack);
  EXPECT_EQ(result.path_source, TrajectoryStateMachine::TrajectorySource::kLocalToEndpoint);
}

TEST(TrajectoryStateMachineTest, FinalPrecisionStopThenFinishedHold) {
  TrajectoryStateMachine state_machine(makeConfig());
  state_machine.setGlobalPath(makeSingleSegmentPath());

  const auto stop = state_machine.update(makeEgo(1.0, 0.0), 0.0);
  EXPECT_EQ(stop.state, TrajectoryStateMachine::PlannerState::kStopping);
  EXPECT_EQ(stop.intent, TrajectoryStateMachine::TrajectoryIntent::kStopCurrent);
  EXPECT_EQ(stop.path_source, TrajectoryStateMachine::TrajectorySource::kActiveSegment);

  const auto finished = state_machine.update(makeEgo(1.0, 0.0), 0.6);
  EXPECT_TRUE(finished.finished);
  EXPECT_EQ(finished.state, TrajectoryStateMachine::PlannerState::kFinishedHold);
  EXPECT_EQ(finished.intent, TrajectoryStateMachine::TrajectoryIntent::kFinishedHold);
}

TEST(TrajectoryStateMachineTest, IntermediateCuspAdvancesToPresteer) {
  TrajectoryStateMachine state_machine(makeConfig());
  state_machine.setGlobalPath(makeTwoSegmentPath());

  const auto stop = state_machine.update(makeEgo(1.0, 0.0), 0.0);
  EXPECT_EQ(stop.intent, TrajectoryStateMachine::TrajectoryIntent::kStopCurrent);
  EXPECT_EQ(stop.active_segment_index, 0U);

  const auto presteer = state_machine.update(makeEgo(1.0, 0.0), 0.6);
  EXPECT_EQ(presteer.intent, TrajectoryStateMachine::TrajectoryIntent::kPresteerNext);
  EXPECT_EQ(presteer.active_segment_index, 1U);
  EXPECT_EQ(presteer.active_direction, -1);
}

TEST(TrajectoryStateMachineTest, TimeoutRequestsEmergencyStopOnActiveSegment) {
  TrajectoryStateMachine state_machine(makeConfig());
  state_machine.setGlobalPath(makeSingleSegmentPath());

  const auto result = state_machine.stopForTimeout(1.0);

  EXPECT_TRUE(result.publish);
  EXPECT_EQ(result.intent, TrajectoryStateMachine::TrajectoryIntent::kEmergencyStop);
  EXPECT_EQ(result.path_source, TrajectoryStateMachine::TrajectorySource::kActiveSegment);
}

TEST(TrajectoryStateMachineTest, ModeChoosesTrajectorySource) {
  TrajectoryStateMachine local(makeConfig(TrajectoryStateMachine::Mode::kLocal));
  local.setGlobalPath(makeSingleSegmentPath());
  EXPECT_EQ(local.update(makeEgo(0.0), 0.0).path_source,
            TrajectoryStateMachine::TrajectorySource::kLocalToEndpoint);

  TrajectoryStateMachine global(makeConfig(TrajectoryStateMachine::Mode::kGlobal));
  global.setGlobalPath(makeSingleSegmentPath());
  EXPECT_EQ(global.update(makeEgo(0.0), 0.0).path_source,
            TrajectoryStateMachine::TrajectorySource::kActiveSegment);
}

}  // namespace
}  // namespace carmaker_planning

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
