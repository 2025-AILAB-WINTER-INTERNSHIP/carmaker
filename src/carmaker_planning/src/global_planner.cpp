/**
 * @file global_planner.cpp
 * @brief Implementation of the global path planner.
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

  State target_goal = goal;
  if (config_.planner.goal_straight_enabled) {
    State sub_goal;
    sub_goal.x = goal.x - config_.planner.goal_straight_distance * config_.planner.goal_straight_direction * std::cos(goal.theta);
    sub_goal.y = goal.y - config_.planner.goal_straight_distance * config_.planner.goal_straight_direction * std::sin(goal.theta);
    sub_goal.theta = goal.theta;
    target_goal = sub_goal;
  }

  // 1. Validate inputs (Bounds and Collision check for start/target_goal)
  result.status = validateInputs(start, target_goal, map);
  if (result.status != PlanningStatus::SUCCESS_OPTIMAL) {
    result.total_time = elapsedSeconds(total_start);
    return result;
  }

  // 2. Run Hybrid A* search (Core Kinodynamic Expansion)
  const auto planning_start = Clock::now();
  result.status = hybrid_astar_->plan(start, target_goal, map);
  result.planning_time = elapsedSeconds(planning_start);
  result.expanded_nodes = hybrid_astar_->getExpandedNodes();
  result.search_iterations = hybrid_astar_->getSearchIterations();

  if (!result.success()) {
    result.total_time = elapsedSeconds(total_start);
    return result;
  }

  // Store raw path from search
  result.raw_path = hybrid_astar_->getPath();
  result.path = result.raw_path;

  // 최종 직선 구간 덧붙임 (포스트 프로세싱 전단계)
  if (config_.planner.goal_straight_enabled && !result.path.empty()) {
    const double dist_to_goal = config_.planner.goal_straight_distance;
    const double resolution = config_.planner.xy_resolution;
    int num_pts = static_cast<int>(std::ceil(dist_to_goal / resolution));

    double last_x = result.path.back().x;
    double last_y = result.path.back().y;
    double last_theta = result.path.back().theta;
    int dir = config_.planner.goal_straight_direction;

    for (int i = 1; i <= num_pts; ++i) {
      double d = std::min(dist_to_goal, i * resolution);
      PathPoint p;
      p.x = last_x + d * std::cos(last_theta) * dir;
      p.y = last_y + d * std::sin(last_theta) * dir;
      p.theta = last_theta;
      p.kappa = 0.0;
      p.direction = dir;
      result.path.push_back(p);
    }
    result.raw_path = result.path;
  }

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

  bool request_smoothing = enable_smoothing_;
  bool request_resampling = enable_resampling_;
  bool request_profiling = enable_velocity_profiling_;

  if (request_profiling && !request_resampling) {
    result.warn("Velocity profiling is enabled but resampling is disabled. Forcing resampling to preserve kinematic consistency!");
    request_resampling = true;
  }

  bool success = post_processor_->process(result, map, current_velocity,
                                          request_smoothing, request_resampling, request_profiling);
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
