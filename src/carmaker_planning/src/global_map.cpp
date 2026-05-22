/**
 * @file global_map.cpp
 * @brief Implementation of unified GlobalMap class.
 */

#include "carmaker_planning/global_map.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <queue>
#include <limits>

namespace carmaker_planning {

GlobalMap::GlobalMap(const GlobalMainConfig& config)
: config_(config.global_costmap)
{
  setVehicleSpec(config.vehicle);
  setLethalThreshold(config_.grid_map.lethal_threshold);
  setUnknownCostFree(config_.grid_map.unknown_cost_free);
  setUnknownValue(static_cast<uint8_t>(config_.grid_map.unknown_value));
}

void GlobalMap::setMeta(int width, int height, double res, double ox, double oy) {
  resolution_ = res; width_ = width; height_ = height; origin_x_ = ox; origin_y_ = oy;
}

void GlobalMap::allocate(uint8_t fill_value) {
  grid_data_.assign(static_cast<size_t>(width_) * height_, fill_value);
}

bool GlobalMap::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  if (resolution_ <= 0.0) return false;
  gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  return isInsideInternal(gx, gy);
}

void GlobalMap::gridToWorld(int gx, int gy, double& wx, double& wy) const {
  wx = origin_x_ + (gx + 0.5) * resolution_;
  wy = origin_y_ + (gy + 0.5) * resolution_;
}

bool GlobalMap::isInside(int gx, int gy) const {
  return isInsideInternal(gx, gy);
}

bool GlobalMap::isInbounds(double x, double y) const { int gx, gy; return worldToGrid(x, y, gx, gy); }

bool GlobalMap::isObstacle(int gx, int gy) const {
  return isObstacleInternal(gx, gy);
}

bool GlobalMap::isObstacleInternal(int gx, int gy) const {
  if (!isInsideInternal(gx, gy)) return true;
  uint8_t val = grid_data_[getIndex(gx, gy)];
  if (val == unknown_value_) return !unknown_cost_free_;
  return val >= static_cast<uint8_t>(lethal_threshold_);
}

bool GlobalMap::checkCollisionPoint(double x, double y) const {
  int gx=-1, gy=-1;
  if (!worldToGrid(x, y, gx, gy)) return true;
  return isObstacleInternal(gx, gy);
}

bool GlobalMap::checkCollision(double x, double y, double theta) const {
  if (grid_data_.empty() || distance_map_.empty()) return true;
  if (!has_vehicle_spec_ || vehicle_spec_.collision_circles.empty()) {
    int gx=-1, gy=-1;
    if (!worldToGrid(x, y, gx, gy)) return true;
    return isObstacleInternal(gx, gy);
  }
  const double c = std::cos(theta), s = std::sin(theta);
  for (size_t i = 0; i < vehicle_spec_.collision_circles.size(); ++i) {
    const auto& circle = vehicle_spec_.collision_circles[i];
    double cx = x + circle.offset * c, cy = y + circle.offset * s;
    int gx=-1, gy=-1;
    if (!worldToGrid(cx, cy, gx, gy)) return true;
    double dist_to_obs = distance_map_[getIndex(gx, gy)];
    if (dist_to_obs < circle.radius) return true;
  }
  return false;
}

void GlobalMap::setCell(int gx, int gy, uint8_t value) {
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  if (!isInsideInternal(gx, gy)) return;
  grid_data_[getIndex(gx, gy)] = value;
  computeEDT();
  bumpVersion();
}

void GlobalMap::updateMap(
  int width, int height, double res, double ox, double oy,
  const std::vector<int8_t>& data)
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);

  if (data.size() != static_cast<size_t>(width) * height) {
    std::cerr << "[GlobalMap] Error: Map size mismatch!" << std::endl;
    return;
  }

  setLethalThreshold(config_.grid_map.lethal_threshold);
  setUnknownCostFree(config_.grid_map.unknown_cost_free);
  setUnknownValue(static_cast<uint8_t>(config_.grid_map.unknown_value));

  setMeta(width, height, res, ox, oy);
  grid_data_.assign(data.begin(), data.end());
  computeEDT();
  bumpVersion();
}

void GlobalMap::setObstacle(int gx, int gy, bool active)
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  if (!isInsideInternal(gx, gy)) {
    return;
  }
  grid_data_[getIndex(gx, gy)] = active ? static_cast<uint8_t>(config_.grid_map.occupancy_value) : 0;
  computeEDT();
  bumpVersion();
}

void GlobalMap::computeEDT() {
  if (width_ <= 0 || height_ <= 0 || grid_data_.empty()) {
    distance_map_.clear();
    return;
  }

  const size_t map_size = static_cast<size_t>(width_) * height_;
  distance_map_.assign(map_size, std::numeric_limits<double>::infinity());

  struct ObstacleRef {
    int nearest_x;
    int nearest_y;
  };
  std::vector<ObstacleRef> nearest_obstacle(map_size, {-1, -1});
  std::queue<int> q;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (isObstacleInternal(x, y)) {
        size_t idx = getIndex(x, y);
        distance_map_[idx] = 0.0;
        nearest_obstacle[idx] = {x, y};
        q.push(static_cast<int>(idx));
      }
    }
  }

  const int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};

  while (!q.empty()) {
    int u_idx = q.front();
    q.pop();

    int ux = u_idx % width_;
    int uy = u_idx / width_;

    ObstacleRef ref = nearest_obstacle[u_idx];

    for (int i = 0; i < 8; ++i) {
      int vx = ux + dx[i];
      int vy = uy + dy[i];

      if (vx >= 0 && vx < width_ && vy >= 0 && vy < height_) {
        size_t v_idx = getIndex(vx, vy);
        double dist_x = (vx - ref.nearest_x) * resolution_;
        double dist_y = (vy - ref.nearest_y) * resolution_;
        double new_dist = std::hypot(dist_x, dist_y);

        if (new_dist < distance_map_[v_idx]) {
          distance_map_[v_idx] = new_dist;
          nearest_obstacle[v_idx] = ref;
          q.push(static_cast<int>(v_idx));
        }
      }
    }
  }
}

} // namespace carmaker_planning
