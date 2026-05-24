/**
 * @file hybrid_astar.cpp
 * @brief Implementation of Hybrid A* (Kinodynamic Search) under unified namespace.
 */

#include "carmaker_planning/hybrid_astar.h"
#include <chrono>
#include <algorithm>
#include <ros/ros.h>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace carmaker_planning
{

// Dijkstra Implementation

void Dijkstra::update(const GlobalMap & map, const State & goal_state, int downsample_scale)
{
  if (downsample_scale < 1) {downsample_scale = 1;}
  scale_ = downsample_scale;

  width_ = (map.getWidth() + scale_ - 1) / scale_;
  height_ = (map.getHeight() + scale_ - 1) / scale_;
  resolution_ = map.getResolution() * scale_;
  origin_x_ = map.getOriginX();
  origin_y_ = map.getOriginY();

  // Reset cost grid
  const size_t size = static_cast<size_t>(width_) * height_;
  if (size <= 0) {return;}
  cost_grid_.assign(size, std::numeric_limits<double>::infinity());

  // Goal index
  int original_gx = -1, original_gy = -1;
  if (!map.worldToGrid(goal_state.x, goal_state.y, original_gx, original_gy)) {
    return; // Outbound
  }
  const int gx = original_gx / scale_;
  const int gy = original_gy / scale_;
  if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {return;}

  std::priority_queue<DioNode, std::vector<DioNode>, std::greater<DioNode>> pq;

  // Init goal
  const size_t goal_idx = getIndex(gx, gy);
  cost_grid_[goal_idx] = 0.0;
  pq.push({static_cast<int>(goal_idx), 0.0});

  // 8-Connectivity movements
  const int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};
  const double edge_cost[8] = {1.0, 1.0, 1.0, 1.0, 1.4142, 1.4142, 1.4142, 1.4142};

  while (!pq.empty()) {
    DioNode current = pq.top();
    pq.pop();

    const int u_idx = current.idx;
    if (current.cost > cost_grid_[u_idx]) {continue;}

    const int ux = u_idx % width_;
    const int uy = u_idx / width_;

    for (int i = 0; i < 8; ++i) {
      const int vx = ux + dx[i];
      const int vy = uy + dy[i];

      if (vx < 0 || vx >= width_ || vy < 0 || vy >= height_) {continue;}

      if (isObstacleInBlock(map, vx, vy)) {
        continue;
      }
      const size_t v_idx = getIndex(vx, vy);
      const double new_cost = current.cost + edge_cost[i] * resolution_;

      if (new_cost < cost_grid_[v_idx]) {
        cost_grid_[v_idx] = new_cost;
        pq.push({static_cast<int>(v_idx), new_cost});
      }
    }
  }
}

bool Dijkstra::isObstacleInBlock(const GlobalMap & map, int dx, int dy) const
{
  const size_t start_x = static_cast<size_t>(dx) * scale_;
  const size_t start_y = static_cast<size_t>(dy) * scale_;

  const size_t map_w = static_cast<size_t>(map.getWidth());
  const size_t map_h = static_cast<size_t>(map.getHeight());
  const size_t scale_sz = static_cast<size_t>(scale_);

  const size_t end_x = std::min(start_x + scale_sz, map_w);
  const size_t end_y = std::min(start_y + scale_sz, map_h);

  for (size_t y = start_y; y < end_y; ++y) {
    for (size_t x = start_x; x < end_x; ++x) {
      if (map.isObstacle(static_cast<int>(x), static_cast<int>(y))) {
        return true;
      }
    }
  }

  return false;
}

double Dijkstra::getH(double x, double y) const
{
  if (cost_grid_.empty()) {return std::numeric_limits<double>::infinity();}
  if (resolution_ <= 0) {return std::numeric_limits<double>::infinity();}

  const double dx = (x - origin_x_);
  const double dy = (y - origin_y_);

  const int gx = static_cast<int>(std::floor(dx / resolution_));
  const int gy = static_cast<int>(std::floor(dy / resolution_));

  if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
    return std::numeric_limits<double>::infinity();
  }

  return cost_grid_[getIndex(gx, gy)];
}

// HeuristicPlanner Implementation

HeuristicPlanner::HeuristicPlanner(const GlobalMainConfig & config)
: config_(config.planner), vehicle_spec_(config.vehicle) {}

void HeuristicPlanner::initialize(const GlobalMap & map, const State & goal)
{
  if (std::abs(goal.x - last_goal_x_) < 1e-4 &&
      std::abs(goal.y - last_goal_y_) < 1e-4 &&
      map.getVersion() == last_map_version_)
  {
    return;
  }

  last_goal_x_ = goal.x;
  last_goal_y_ = goal.y;
  last_map_version_ = map.getVersion();

  dijkstra_.update(map, goal, config_.heuristic_scale);
}

double HeuristicPlanner::calcH(const GlobalNode3D & node, const State & goal)
{
  double h_obs = dijkstra_.getH(node.x, node.y);
  if (std::isinf(h_obs)) {
    return std::numeric_limits<double>::infinity();
  }

  double dx = node.x - goal.x;
  double dy = node.y - goal.y;
  double d = std::hypot(dx, dy);
  double rho = vehicle_spec_.min_turning_radius;

  if (h_obs >= d + 2.0 * PI * rho) {
    return h_obs;
  }

  State start(node.x, node.y, node.theta);
  double h_kin = ReedsSheppCurve::getShortestLength(start, goal, rho);

  return std::max(h_obs, h_kin);
}

// VersionedHashMap Implementation

void VersionedHashMap::resize(size_t capacity)
{
  size_t pow2 = 1;
  while (pow2 < capacity) {
    pow2 <<= 1;
  }
  buckets_.assign(pow2, Bucket());
  mask_ = pow2 - 1;
  version_ = 1;
}

void VersionedHashMap::clear()
{
  version_++;
  if (version_ == 0) {
    version_ = 1;
    for (auto & bucket : buckets_) {
      bucket.version = 0;
    }
  }
}

GlobalNode3D* VersionedHashMap::find(size_t key) const
{
  if (buckets_.empty()) {
    return nullptr;
  }
  size_t idx = hash(key) & mask_;
  size_t count = 0;
  const size_t sz = buckets_.size();
  while (count < sz) {
    const auto & bucket = buckets_[idx];
    if (bucket.version != version_) {
      return nullptr;
    }
    if (bucket.key == key) {
      return bucket.node;
    }
    idx = (idx + 1) & mask_;
    count++;
  }
  return nullptr;
}

void VersionedHashMap::insert(size_t key, GlobalNode3D* node)
{
  if (buckets_.empty()) {
    return;
  }
  size_t idx = hash(key) & mask_;
  size_t count = 0;
  const size_t sz = buckets_.size();
  while (count < sz) {
    auto & bucket = buckets_[idx];
    if (bucket.version != version_ || bucket.key == key) {
      bucket.key = key;
      bucket.node = node;
      bucket.version = version_;
      return;
    }
    idx = (idx + 1) & mask_;
    count++;
  }
}

// HybridAStar Implementation

HybridAStar::HybridAStar(const GlobalMainConfig & config, const std::string& logger_name)
: config_(config.planner), vehicle_spec_(config.vehicle), heuristic_planner_(config)
, logger_name_(logger_name)
{
  theta_res_ = config.planner.theta_resolution;
  angle_size_ = static_cast<int>(std::ceil(2.0 * PI / theta_res_));
  nodes_grid_.resize(1048576);
}

PlanningStatus HybridAStar::plan(const State & start, const State & goal, GlobalMap & map)
{
  resetSearch(map);

  GlobalNode3D * start_node = initializeStartNode(start, goal, map);
  if (!start_node) {
    ROS_ERROR_NAMED(logger_name_, "Start node initialization failed (collision or out of bounds)");
    return PlanningStatus::FAILURE_COLLISION;
  }

  GlobalNode3D * best_node = start_node;
  double min_h = start_node->h_cost;

  int iter = 0;
  const int max_iter = config_.max_iterations;
  const double max_time = config_.max_planning_time;
  auto start_clock = std::chrono::steady_clock::now();

  while (!open_set_.empty() && iter < max_iter) {
    iter++;

    if (iter % 100 == 0) {
      auto current_clock = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(current_clock - start_clock).count() > max_time) {
        ROS_WARN_NAMED(logger_name_, "Timeout reached (%.2fs)", max_time);
        tracePath(best_node);
        return PlanningStatus::FAILURE_TIMEOUT;
      }
    }

    GlobalNode3D * current = open_set_.top();
    open_set_.pop();

    if (current->is_closed) {continue;}
    current->is_closed = true;

    if (current->h_cost < min_h) {
      min_h = current->h_cost;
      best_node = current;
    }

    double dist_to_goal = dist(current->x, current->y, goal.x, goal.y);
    double angle_to_goal = std::abs(wrap_to_pi(current->theta - goal.theta));

    if (dist_to_goal < config_.goal_tolerance.xy && angle_to_goal < config_.goal_tolerance.theta) {
      tracePath(current);
      return PlanningStatus::SUCCESS_OPTIMAL;
    }

    if (iter % config_.analytic_expansion_ratio == 0) {
      double dummy_f;
      if (tryAnalyticExpansion(current, goal, map, dummy_f)) {
        return PlanningStatus::SUCCESS_OPTIMAL;
      }
    }

    expandNode(current, goal, map);
  }

  ROS_WARN_NAMED(logger_name_, "Goal not reached. Returning Best-Effort path (min_h: %.2fm)", min_h);
  tracePath(best_node);

  if (iter >= max_iter) return PlanningStatus::FAILURE_TIMEOUT;
  if (open_set_.empty()) return PlanningStatus::FAILURE_NO_PATH;
  return PlanningStatus::SUCCESS_BEST_EFFORT;
}

void HybridAStar::resetSearch(GlobalMap & map)
{
  path_.clear();
  node_pool_.clear();
  while (!open_set_.empty()) {open_set_.pop();}

  width_ = map.getWidth();
  height_ = map.getHeight();

  nodes_grid_.clear();
}

GlobalNode3D * HybridAStar::initializeStartNode(
  const State & start, const State & goal, GlobalMap & map)
{
  heuristic_planner_.initialize(map, goal);

  GlobalNode3D temp_start;
  temp_start.x = start.x;
  temp_start.y = start.y;
  temp_start.theta = start.theta;
  double start_h = heuristic_planner_.calcH(temp_start, goal);

  if (std::isinf(start_h)) {return nullptr;}

  GlobalNode3D * start_node = createNode(
    start.x, start.y, start.theta, 0.0, start_h, nullptr, 0.0, 1);
  open_set_.push(start_node);

  size_t start_t_idx = getThetaIndex(start.theta);
  size_t start_g_idx = getGridIndex(
    static_cast<int>(std::floor((start_node->x - map.getOriginX()) / map.getResolution())),
    static_cast<int>(std::floor((start_node->y - map.getOriginY()) / map.getResolution())),
    static_cast<int>(start_t_idx));

  if (start_g_idx == std::numeric_limits<size_t>::max()) {
    return nullptr;
  }

  nodes_grid_.insert(start_g_idx, start_node);

  return start_node;
}

void HybridAStar::expandNode(GlobalNode3D * current, const State & goal, GlobalMap & map)
{
  int num_steer = config_.next_node_num;
  double max_steer = vehicle_spec_.max_steer_angle;
  double d_steer = (num_steer > 1) ? (2.0 * max_steer / (num_steer - 1)) : 0.0;
  const std::vector<int> directions = {1, -1};

  // 1. Gather all candidates
  struct Candidate {
    int dir;
    double steer;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(2 * num_steer);
  for (int dir : directions) {
    for (int i = 0; i < num_steer; ++i) {
      candidates.push_back({dir, -max_steer + i * d_steer});
    }
  }

  // 2. Evaluate candidates in parallel
  struct ValidNode {
    double x, y, theta;
    double g, h;
    double steer;
    int dir;
    size_t g_idx;
  };
  std::vector<ValidNode> valid_nodes(candidates.size());
  std::vector<bool> is_valid(candidates.size(), false);

  #pragma omp parallel for schedule(static, 1)
  for (size_t i = 0; i < candidates.size(); ++i) {
    int dir = candidates[i].dir;
    double steer = candidates[i].steer;
    double step_dist = config_.step_size * dir;

    double next_x = 0.0;
    double next_y = 0.0;
    double next_theta = 0.0;

    if (std::abs(steer) < 1e-3) {
      next_x = current->x + step_dist * std::cos(current->theta);
      next_y = current->y + step_dist * std::sin(current->theta);
      next_theta = current->theta;
    } else {
      const double kappa = std::tan(steer) / vehicle_spec_.wheelbase;
      const double d_theta = step_dist * kappa;
      next_theta = wrap_to_pi(current->theta + d_theta);

      const double radius = 1.0 / kappa;
      next_x = current->x + radius * (std::sin(next_theta) - std::sin(current->theta));
      next_y = current->y - radius * (std::cos(next_theta) - std::cos(current->theta));
    }

    if (map.checkCollision(next_x, next_y, next_theta)) {continue;}

    size_t t_idx = getThetaIndex(next_theta);
    size_t g_idx = getGridIndex(
      static_cast<int>(std::floor((next_x - map.getOriginX()) / map.getResolution())),
      static_cast<int>(std::floor((next_y - map.getOriginY()) / map.getResolution())),
      static_cast<int>(t_idx));

    if (g_idx == std::numeric_limits<size_t>::max()) {
      continue;
    }

    double step_cost = std::abs(step_dist);
    if (dir == -1) {
      step_cost *= config_.weights.reverse;
    }
    if (dir != current->direction) {
      step_cost += config_.weights.change_dir;
    }
    if (std::abs(steer) > 0.01) {
      const double turn_factor = config_.weights.turn * (std::abs(steer) / max_steer);
      step_cost *= (1.0 + turn_factor);
    }

    double next_g = current->g_cost + step_cost;

    GlobalNode3D temp_next;
    temp_next.x = next_x; temp_next.y = next_y; temp_next.theta = next_theta;

    double next_h = heuristic_planner_.calcH(temp_next, goal);
    if (std::isinf(next_h)) {continue;}

    valid_nodes[i] = {next_x, next_y, next_theta, next_g, next_h, steer, dir, g_idx};
    is_valid[i] = true;
  }

  // 3. Sequentially insert valid nodes into A* open set and nodes grid (thread-safe)
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (!is_valid[i]) {
      continue;
    }
    const auto & vn = valid_nodes[i];
    GlobalNode3D* existing = nodes_grid_.find(vn.g_idx);
    if (existing == nullptr || existing->g_cost > vn.g) {
      GlobalNode3D * next_node = createNode(
        vn.x, vn.y, vn.theta, vn.g, vn.h, current, vn.steer,
        vn.dir);

      nodes_grid_.insert(vn.g_idx, next_node);
      open_set_.push(next_node);
    }
  }
}

GlobalNode3D * HybridAStar::createNode(
  double x, double y, double theta, double g,
  double h, const GlobalNode3D * parent, double steer,
  int dir)
{
  node_pool_.emplace_back();
  GlobalNode3D & node = node_pool_.back();

  node.x = x;
  node.y = y;
  node.theta = theta;
  node.g_cost = g;
  node.h_cost = h * config_.weights.heuristic;

  node.parent = parent;
  node.steering = steer;
  node.direction = dir;
  node.is_closed = false;

  return &node;
}

bool HybridAStar::tryAnalyticExpansion(
  const GlobalNode3D * current, const State & goal,
  const GlobalMap & map,
  double & /*best_f_cost*/)
{
  double rho = vehicle_spec_.min_turning_radius;
  State start_st(current->x, current->y, current->theta);

  RSPath rs_path;
  bool found = ReedsSheppCurve::calculatePath(start_st, goal, rho, rs_path);
  if (!found) {
    return false;
  }

  // 1. Dynamic step size collision check along Reeds-Shepp segments
  auto check_collision_dynamic = [&](const RSPath& path) {
    const double inv_rho = 1.0 / path.rho;
    State current_state = start_st;

    for (size_t i = 0; i < path.seg_lengths.size(); ++i) {
      double seg_len_m = path.seg_lengths[i] * path.rho;
      int dir = path.seg_dir[i];
      int steer = path.seg_steer[i];

      double seg_s = 0.0;
      while (seg_s < seg_len_m) {
        double margin = map.getCollisionSafetyMargin(current_state.x, current_state.y, current_state.theta);
        if (margin < 0.0) {
          return true; // Collision
        }

        // Stepping dynamically: safely skip up to margin distance.
        // Cap step size between 0.05m and 0.5m.
        double step = std::clamp(margin, 0.05, 0.5);
        if (seg_s + step > seg_len_m) {
          step = seg_len_m - seg_s;
        }

        seg_s += step;
        if (steer == 0) { // Straight
          current_state.x += dir * step * std::cos(current_state.theta);
          current_state.y += dir * step * std::sin(current_state.theta);
        } else { // Turn
          const double d_theta = (dir * steer * step) * inv_rho;
          const double theta_new = wrap_to_pi(current_state.theta + d_theta);
          current_state.x += path.rho * steer * (std::sin(theta_new) - std::sin(current_state.theta));
          current_state.y -= path.rho * steer * (std::cos(theta_new) - std::cos(current_state.theta));
          current_state.theta = theta_new;
        }
      }
    }
    return map.checkCollision(current_state.x, current_state.y, current_state.theta);
  };

  if (check_collision_dynamic(rs_path)) {
    return false;
  }

  // 2. Only sample path at a moderate step size if it is collision-free
  double sample_step = std::max(map.getResolution() * 2.0, 0.15);
  ReedsSheppCurve::sample(rs_path, start_st, sample_step);

  tracePath(current);

  for (const auto & p : rs_path.points) {
    if (!path_.empty()) {
      double d = dist(path_.back().x, path_.back().y, p.x, p.y);
      if (d < 1e-3) {continue;}
    }
    path_.push_back(p);
  }

  return true;
}

void HybridAStar::tracePath(const GlobalNode3D * node)
{
  std::vector<PathPoint> blind_path;
  const GlobalNode3D * curr = node;

  while (curr != nullptr) {
    PathPoint p(curr->x, curr->y, curr->theta);
    p.direction = curr->direction;
    blind_path.push_back(p);
    curr = curr->parent;
  }

  std::reverse(blind_path.begin(), blind_path.end());
  path_ = std::move(blind_path);
}

size_t HybridAStar::getThetaIndex(double theta) const
{
  double n_theta = wrap_to_2pi(theta);
  size_t idx = static_cast<size_t>(n_theta / theta_res_);
  return idx % angle_size_;
}

size_t HybridAStar::getGridIndex(int x_idx, int y_idx, int th_idx) const
{
  if (x_idx < 0 || x_idx >= width_ ||
      y_idx < 0 || y_idx >= height_ ||
      th_idx < 0 || th_idx >= angle_size_)
  {
    return std::numeric_limits<size_t>::max();
  }

  return (static_cast<size_t>(th_idx) * height_ * width_) + (static_cast<size_t>(y_idx) * width_) +
        static_cast<size_t>(x_idx);
}

std::vector<State> HybridAStar::getSearchTree() const
{
  std::vector<State> tree;
  tree.reserve(node_pool_.size());

  for (const auto & node : node_pool_) {
    if (node.is_closed) {
      tree.emplace_back(node.x, node.y, node.theta);
    }
  }
  return tree;
}

std::vector<std::pair<State, State>> HybridAStar::getTreeBranches() const
{
  std::vector<std::pair<State, State>> branches;
  branches.reserve(node_pool_.size());

  for (const auto & node : node_pool_) {
    if (node.parent != nullptr && node.is_closed) {
      State p_state(node.parent->x, node.parent->y, node.parent->theta);
      State c_state(node.x, node.y, node.theta);
      branches.push_back({p_state, c_state});
    }
  }
  return branches;
}

} // namespace carmaker_planning
