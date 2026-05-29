/**
 * @file global_planner.cpp
 * @brief Implementation of the Global Planner orchestrator.
 */

#include "carmaker_planning/global_planner.h"
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

namespace carmaker_planning {

GlobalPlanner::GlobalPlanner(const GlobalMainConfig& config, const std::string& logger_name)
  : config_(config), logger_name_(logger_name)
{
  hybrid_astar_ = std::make_unique<HybridAStar>(config, logger_name);
  smoother_ = std::make_unique<PathSmoother>(config);
  resampler_ = std::make_unique<PathResampler>(config);
  velocity_profiler_ = std::make_unique<VelocityProfiler>(config);
}

void GlobalPlanner::updateConfig(const GlobalMainConfig& config)
{
  config_ = config;

  hybrid_astar_ = std::make_unique<HybridAStar>(config, logger_name_);
  smoother_ = std::make_unique<PathSmoother>(config);
  resampler_ = std::make_unique<PathResampler>(config);
  velocity_profiler_ = std::make_unique<VelocityProfiler>(config);
}

GlobalPlanningResult GlobalPlanner::plan(
  const State& start,
  const State& goal,
  GlobalMap& map,
  double current_velocity)
{
  GlobalPlanningResult result;
  const auto total_start = Clock::now();

  // 1. Validate inputs (Bounds and Collision check for start/goal)
  result.status = validateInputs(start, goal, map);
  if (result.status != PlanningStatus::SUCCESS_OPTIMAL) {
    result.total_time = elapsedSeconds(total_start);
    return result;
  }

  // 2. Run Hybrid A* search (Core Kinodynamic Expansion)
  const auto planning_start = Clock::now();
  result.status = hybrid_astar_->plan(start, goal, map);
  result.planning_time = elapsedSeconds(planning_start);

  if (!result.success()) {
    result.total_time = elapsedSeconds(total_start);
    return result;
  }

  // Store raw path from search
  result.raw_path = hybrid_astar_->getPath();
  result.path = result.raw_path;

  // 3. Post-processing Pipeline (Smoothing -> Resampling -> Profiling)
  if (!postProcess(result, map, current_velocity)) {
    ROS_WARN_NAMED(logger_name_, "Global Planner: Post-processing pipeline failed.");
    if (result.path.empty()) {
      result.status = PlanningStatus::FAILURE_NO_PATH;
    }
  }

  result.total_time = elapsedSeconds(total_start);
  return result;
}

GlobalPlanningResult GlobalPlanner::planGeometric(
  const State& start,
  const State& goal,
  GlobalMap& map)
{
  const bool was_enabled = enable_velocity_profiling_;
  enable_velocity_profiling_ = false;

  auto result = plan(start, goal, map, 0.0);

  enable_velocity_profiling_ = was_enabled;
  return result;
}

PlanningStatus GlobalPlanner::validateInputs(
  const State& start,
  const State& goal,
  const GlobalMap& map) const
{
  if (map.getWidth() <= 0 || map.getHeight() <= 0) {
    return PlanningStatus::FAILURE_INVALID_MAP;
  }

  if (!isValidState(start, map)) {
    ROS_WARN_NAMED(logger_name_, "[Global] Start state in collision or out of bounds (%.2f, %.2f)", start.x, start.y);
    return PlanningStatus::FAILURE_COLLISION;
  }

  if (!isValidState(goal, map)) {
    ROS_WARN_NAMED(logger_name_, "[Global] Goal state in collision or out of bounds (%.2f, %.2f)", goal.x, goal.y);
    return PlanningStatus::FAILURE_COLLISION;
  }

  return PlanningStatus::SUCCESS_OPTIMAL;
}

bool GlobalPlanner::isValidState(const State& state, const GlobalMap& map) const
{
  const double origin_x = map.getOriginX();
  const double origin_y = map.getOriginY();
  const double width_m = map.getWidth() * map.getResolution();
  const double height_m = map.getHeight() * map.getResolution();

  if (state.x < origin_x || state.x >= origin_x + width_m ||
      state.y < origin_y || state.y >= origin_y + height_m) {
    return false;
  }

  if (map.checkCollision(state.x, state.y, state.theta)) {
    return false;
  }

  return true;
}

bool GlobalPlanner::postProcess(GlobalPlanningResult& result, const GlobalMap& map, double current_velocity)
{
  if (result.path.empty()) {
    return false;
  }

  // 1. Smoothing
  if (enable_smoothing_) {
    const auto start = Clock::now();
    smoother_->smooth(result.path, map);
    result.smoothing_time = elapsedSeconds(start);
  }

  // 2. Resampling
  if (enable_resampling_) {
    const auto start = Clock::now();
    Path resampled_path;
    if (resampler_->resample(result.path, resampled_path)) {
      result.path = std::move(resampled_path);
    }
    result.resampling_time = elapsedSeconds(start);
  }

  // 3. Velocity profiling
  if (enable_velocity_profiling_) {
    const auto start = Clock::now();
    if (!velocity_profiler_->profile(result.path, current_velocity)) {
      // Profiling failed
    }
    result.profiling_time = elapsedSeconds(start);
  }

  return true;
}

std::vector<State> GlobalPlanner::getSearchTree() const
{
  if (hybrid_astar_) {
    return hybrid_astar_->getSearchTree();
  }
  return {};
}

std::vector<std::pair<State, State>> GlobalPlanner::getTreeBranches() const
{
  if (hybrid_astar_) {
    return hybrid_astar_->getTreeBranches();
  }
  return {};
}

} // namespace carmaker_planning
