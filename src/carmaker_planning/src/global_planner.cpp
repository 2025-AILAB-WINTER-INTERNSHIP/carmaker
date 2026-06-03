/**
 * @file global_planner.cpp
 * @brief Implementation of the Global Planner orchestrator.
 */

#include "carmaker_planning/global_planner.h"
#include <algorithm>
#include <cmath>

namespace carmaker_planning {

GlobalPlanner::GlobalPlanner(const GlobalMainConfig& config, const std::string& logger_name)
  : config_(config), logger_name_(logger_name)
{
  hybrid_astar_ = std::make_unique<HybridAStar>(config, logger_name);
  post_processor_ = std::make_unique<PostProcessor>(config);
}

void GlobalPlanner::updateConfig(const GlobalMainConfig& config)
{
  config_ = config;

  hybrid_astar_ = std::make_unique<HybridAStar>(config, logger_name_);
  post_processor_ = std::make_unique<PostProcessor>(config);
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
    return PlanningStatus::FAILURE_COLLISION_START;
  }

  if (!isValidState(goal, map)) {
    return PlanningStatus::FAILURE_COLLISION_GOAL;
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

  bool run_smoothing = enable_smoothing_;
  bool run_resampling = enable_resampling_;
  bool run_profiling = enable_velocity_profiling_;

  if (run_profiling && !run_resampling) {
    result.warn("Velocity profiling is enabled but resampling is disabled. Forcing resampling to preserve kinematic consistency!");
    run_resampling = true;
  }

  bool success = post_processor_->process(result, map, current_velocity,
                                          run_smoothing, run_resampling, run_profiling);
  if (!success) {
    result.status = PlanningStatus::FAILURE_POST_PROCESSING;
  }
  return success;
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
