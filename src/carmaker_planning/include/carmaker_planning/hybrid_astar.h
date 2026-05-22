/**
 * @file hybrid_astar.h
 * @brief Hybrid A* Kinodynamic Search with Reverse Support.
 */
#ifndef CARMAKER_PLANNING_HYBRID_ASTAR_H
#define CARMAKER_PLANNING_HYBRID_ASTAR_H

#include <deque>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <limits>
#include <unordered_map>

#include "carmaker_planning/global_map.h"
#include "carmaker_planning/types.h"
#include "carmaker_planning/reeds_shepp.h"

namespace carmaker_planning {

class Dijkstra {
public:
  Dijkstra() = default;
  ~Dijkstra() = default;
  Dijkstra(const Dijkstra&) = delete;
  Dijkstra& operator=(const Dijkstra&) = delete;

  void update(const GlobalMap& map, const State& goal_state, int downsample_scale=1);
  double getH(double x, double y) const;

  const std::vector<double>& getCostGrid() const { return cost_grid_; }
  int getWidth() const { return width_; }
  int getHeight() const { return height_; }
  double getResolution() const { return resolution_; }

private:
  int width_=0, height_=0, scale_=1;
  double resolution_=0.05, origin_x_=0.0, origin_y_=0.0;
  std::vector<double> cost_grid_;

  inline size_t getIndex(int x, int y) const {
    return static_cast<size_t>(y) * width_ + static_cast<size_t>(x);
  }
  struct DioNode { int idx; double cost; bool operator>(const DioNode& o) const { return cost > o.cost; } };
  bool isObstacleInBlock(const GlobalMap& map, int dx, int dy) const;
};

class HeuristicPlanner {
public:
  explicit HeuristicPlanner(const GlobalMainConfig& config);
  HeuristicPlanner(const HeuristicPlanner&) = delete;
  HeuristicPlanner& operator=(const HeuristicPlanner&) = delete;

  void initialize(const GlobalMap& map, const State& goal);
  double calcH(const GlobalNode3D& node, const State& goal);

private:
  GlobalPlannerConfig config_;
  VehicleSpec vehicle_spec_;
  Dijkstra dijkstra_;
  double last_goal_x_ = -1e9, last_goal_y_ = -1e9;
  long long last_map_version_ = -1;
};

class HybridAStar {
public:
  explicit HybridAStar(const GlobalMainConfig& config,
                        const std::string& logger_name = "HybridAStar");
  ~HybridAStar() = default;
  HybridAStar(const HybridAStar&) = delete;
  HybridAStar& operator=(const HybridAStar&) = delete;

  PlanningStatus plan(const State& start, const State& goal, GlobalMap& map);

  const Path& getPath() const { return path_; }
  std::vector<State> getSearchTree() const;
  std::vector<std::pair<State, State>> getTreeBranches() const;

private:
  void resetSearch(GlobalMap& map);
  GlobalNode3D* initializeStartNode(const State& start, const State& goal, GlobalMap& map);
  void expandNode(GlobalNode3D* current, const State& goal, GlobalMap& map);
  GlobalNode3D* createNode(double x, double y, double theta, double g, double h,
                            const GlobalNode3D* parent, double steer, int dir);
  bool tryAnalyticExpansion(const GlobalNode3D* current, const State& goal,
                             const GlobalMap& map, double& best_f);
  void tracePath(const GlobalNode3D* node);
  size_t getThetaIndex(double theta) const;
  size_t getGridIndex(int x, int y, int th) const;

  GlobalPlannerConfig config_;
  VehicleSpec vehicle_spec_;
  HeuristicPlanner heuristic_planner_;
  std::string logger_name_;

  std::unordered_map<size_t, GlobalNode3D*> nodes_grid_;
  std::deque<GlobalNode3D> node_pool_;
  std::priority_queue<GlobalNode3D*, std::vector<GlobalNode3D*>, GlobalNode3D::Greater> open_set_;

  Path path_;
  int width_=0, height_=0, angle_size_=0;
  double theta_res_;
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_HYBRID_ASTAR_H
