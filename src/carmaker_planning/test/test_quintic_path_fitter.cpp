#include <gtest/gtest.h>

#include <cmath>

#include "carmaker_planning/local_planner.h"

namespace carmaker_planning {
namespace {

PostProcessConfig makePostProcessConfig() {
  PostProcessConfig config;
  config.smoother.enabled = true;
  config.smoother.collision_check_enabled = false;
  config.smoother.weight_data = 0.2;
  config.smoother.weight_smooth = 0.35;
  config.smoother.tolerance = 1e-3;
  config.smoother.max_iterations = 20;
  config.resampler.enabled = true;
  config.resampler.resolution = 0.1;
  config.validator.enabled = false;
  config.validator.time_tolerance_ms = 10.0;
  config.validator.yaw_tolerance_rad = 0.1;
  config.profiler.enabled = false;
  config.profiler.max_vel = 1.0;
  config.profiler.max_accel = 1.0;
  config.profiler.max_decel = 1.0;
  config.profiler.max_jerk = 1.0;
  config.profiler.max_steer_vel = 1.0;
  config.profiler.max_lat_acc = 1.0;
  config.profiler.goal_vel = 0.0;
  config.profiler.gear_shift_duration = 1.0;
  config.profiler.min_velocity_denominator = 0.02;
  config.visualization.arrow_spacing_meters = 0.2;
  return config;
}

PathPoint makePathPoint(double x, double y, double theta, double kappa, int direction) {
  PathPoint pt;
  pt.x = x;
  pt.y = y;
  pt.theta = theta;
  pt.kappa = kappa;
  pt.direction = direction;
  return pt;
}

TEST(QuinticPathFitterTest, ShortDistanceReturnsStraightFallback) {
  State ego;
  ego.x = 1.0;
  ego.y = 2.0;
  ego.theta = 0.75;

  PathPoint target;
  target.x = ego.x + 1e-4;
  target.y = ego.y - 1e-4;
  target.theta = -1.2;
  target.direction = -1;
  target.kappa = 0.3;

  QuinticPathFitter fitter;
  const Path path = fitter.fit(ego, 0.0, target, 0.1);

  ASSERT_GE(path.size(), 2U);
  EXPECT_DOUBLE_EQ(path.front().x, ego.x);
  EXPECT_DOUBLE_EQ(path.front().y, ego.y);
  EXPECT_DOUBLE_EQ(path.front().theta, ego.theta);
  EXPECT_DOUBLE_EQ(path.front().s, 0.0);

  EXPECT_DOUBLE_EQ(path.back().x, target.x);
  EXPECT_DOUBLE_EQ(path.back().y, target.y);
  EXPECT_DOUBLE_EQ(path.back().theta, target.theta);
  EXPECT_DOUBLE_EQ(path.back().kappa, target.kappa);
  EXPECT_GT(path.back().s, 0.0);

  for (const auto& point : path) {
    EXPECT_EQ(point.direction, -1);
    EXPECT_DOUBLE_EQ(point.v, 0.0);
    EXPECT_DOUBLE_EQ(point.a, 0.0);
    EXPECT_DOUBLE_EQ(point.t, 0.0);
  }
}

TEST(QuinticPathFitterTest, ReverseFitUsesBackwardTravelTangent) {
  State ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.theta = 0.0;

  PathPoint target;
  target.x = -5.0;
  target.y = 0.0;
  target.theta = 0.0;
  target.direction = -1;
  target.kappa = 0.0;

  QuinticPathFitter fitter;
  const Path path = fitter.fit(ego, 0.0, target, 0.1);

  ASSERT_GT(path.size(), 2U);
  EXPECT_EQ(path.front().direction, -1);
  EXPECT_EQ(path.back().direction, -1);
  EXPECT_NEAR(path.front().theta, ego.theta, 1e-9);
  EXPECT_NEAR(path.back().theta, target.theta, 1e-9);
  EXPECT_LT(path[1].x, path.front().x);
  EXPECT_NEAR(path[1].y, path.front().y, 1e-6);
  EXPECT_NEAR(path.back().x, target.x, 1e-9);
  EXPECT_NEAR(path.back().y, target.y, 1e-9);
}

TEST(LocalTrajectoryPlannerTest, PostProcessingPreservesBoundaryYawAndKappa) {
  LocalTrajectoryPlanner planner;
  planner.configure(makePostProcessConfig(), 2.97, 8.5);

  LocalTrajectoryPlanner::PlanRequest request;
  request.ego.x = 0.0;
  request.ego.y = 0.0;
  request.ego.theta = 0.4;
  request.ego.v = 0.0;
  request.start_kappa = 0.15;
  request.target.x = 2.0;
  request.target.y = 1.0;
  request.target.theta = 1.0;
  request.target.kappa = 0.2;
  request.target.direction = 1;
  request.start_vel = 0.0;

  const LocalPlanningResult result = planner.planToEndpoint(request);

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.path.empty());
  EXPECT_NEAR(result.path.front().x, request.ego.x, 1e-9);
  EXPECT_NEAR(result.path.front().y, request.ego.y, 1e-9);
  EXPECT_NEAR(result.path.front().theta, request.ego.theta, 1e-9);
  EXPECT_NEAR(result.path.front().kappa, request.start_kappa, 1e-9);
  EXPECT_NEAR(result.path.back().x, request.target.x, 1e-9);
  EXPECT_NEAR(result.path.back().y, request.target.y, 1e-9);
  EXPECT_NEAR(result.path.back().theta, request.target.theta, 1e-9);
  EXPECT_NEAR(result.path.back().kappa, request.target.kappa, 1e-9);
}

TEST(PostProcessorTest, PreservesCuspBoundaryYawAndKappa) {
  const PathPoint start = makePathPoint(0.0, 0.0, 0.2, 0.11, 1);
  const PathPoint cusp_in = makePathPoint(1.0, 0.0, 0.6, 0.12, 1);
  const PathPoint cusp_out = makePathPoint(1.0, 0.0, -2.4, -0.13, -1);
  const PathPoint end = makePathPoint(0.0, -1.0, -1.5, -0.14, -1);

  LocalPlanningResult result;
  result.path = {start, cusp_in, cusp_out, end};

  PostProcessor processor(makePostProcessConfig(), 2.97, 8.5);
  ASSERT_TRUE(processor.process(result,
                                /*start_vel=*/0.0,
                                /*request_smoothing=*/true,
                                /*request_resampling=*/true,
                                /*request_profiling=*/false));

  const auto find_boundary = [&](const PathPoint& expected) -> const PathPoint* {
    for (const auto& point : result.path) {
      if (std::abs(point.x - expected.x) < 1e-9 &&
          std::abs(point.y - expected.y) < 1e-9 &&
          point.direction == expected.direction &&
          std::abs(point.theta - expected.theta) < 1e-9 &&
          std::abs(point.kappa - expected.kappa) < 1e-9) {
        return &point;
      }
    }
    return nullptr;
  };

  EXPECT_NE(find_boundary(start), nullptr);
  EXPECT_NE(find_boundary(cusp_in), nullptr);
  EXPECT_NE(find_boundary(cusp_out), nullptr);
  EXPECT_NE(find_boundary(end), nullptr);
}

}  // namespace
}  // namespace carmaker_planning

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
