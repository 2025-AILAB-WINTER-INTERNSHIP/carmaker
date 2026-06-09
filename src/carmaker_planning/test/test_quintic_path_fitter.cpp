#include <gtest/gtest.h>

#include <cmath>

#include "carmaker_planning/local_planner.h"

namespace carmaker_planning {
namespace {

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
  EXPECT_DOUBLE_EQ(path.back().theta, ego.theta);
  EXPECT_GT(path.back().s, 0.0);

  for (const auto& point : path) {
    EXPECT_EQ(point.direction, -1);
    EXPECT_DOUBLE_EQ(point.kappa, 0.0);
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

}  // namespace
}  // namespace carmaker_planning

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
