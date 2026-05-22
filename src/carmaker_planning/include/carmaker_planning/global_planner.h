/**
 * @file global_planner.h
 * @brief Orchestrator class for Global Path Planning (Hybrid A*, smoothing, profiling).
 */
#ifndef CARMAKER_PLANNING_GLOBAL_PLANNER_H
#define CARMAKER_PLANNING_GLOBAL_PLANNER_H

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "carmaker_planning/types.h"
#include "carmaker_planning/global_map.h"
#include "carmaker_planning/hybrid_astar.h"
#include "carmaker_planning/post_processing.h"

namespace carmaker_planning {

class GlobalPlanner {
public:
  explicit GlobalPlanner(const GlobalMainConfig& config,
                         const std::string& logger_name = "GlobalPlanner");
  ~GlobalPlanner() = default;
  GlobalPlanner(const GlobalPlanner&) = delete;
  GlobalPlanner& operator=(const GlobalPlanner&) = delete;

  GlobalPlanningResult plan(const State& start, const State& goal,
                            GlobalMap& map, double current_velocity = 0.0);
  GlobalPlanningResult planGeometric(const State& start, const State& goal, GlobalMap& map);
  void updateConfig(const GlobalMainConfig& config);

  std::vector<State> getSearchTree() const;
  std::vector<std::pair<State, State>> getTreeBranches() const;
  bool isValidState(const State& state, const GlobalMap& map) const;

  void enableSmoothing(bool e) { enable_smoothing_ = e; }
  void enableResampling(bool e) { enable_resampling_ = e; }
  void enableVelocityProfiling(bool e) { enable_velocity_profiling_ = e; }

private:
  PlanningStatus validateInputs(const State& start, const State& goal, const GlobalMap& map) const;
  bool postProcess(GlobalPlanningResult& result, const GlobalMap& map, double current_velocity);

  GlobalMainConfig config_;
  std::string logger_name_;

  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  double elapsedSeconds(TimePoint t) const {
    return std::chrono::duration<double>(Clock::now() - t).count();
  }

  std::unique_ptr<HybridAStar> hybrid_astar_;
  std::unique_ptr<PathSmoother> smoother_;
  std::unique_ptr<PathResampler> resampler_;
  std::unique_ptr<VelocityProfiler> velocity_profiler_;

  bool enable_smoothing_ = true;
  bool enable_resampling_ = true;
  bool enable_velocity_profiling_ = true;
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_GLOBAL_PLANNER_H
