/**
 * @file global_map.h
 * @brief Unified GlobalMap class consolidating GridMap and GlobalMap.
 */
#ifndef CARMAKER_PLANNING_GLOBAL_MAP_H
#define CARMAKER_PLANNING_GLOBAL_MAP_H

#include <cstdint>
#include <cmath>
#include <mutex>
#include <vector>
#include <utility>
#include "carmaker_planning/types.h"

namespace carmaker_planning {

class GlobalMap {
public:
  explicit GlobalMap(const GlobalMainConfig& config);
  ~GlobalMap() = default;
  GlobalMap(const GlobalMap&) = delete;
  GlobalMap& operator=(const GlobalMap&) = delete;

  int getWidth() const { return width_; }
  int getHeight() const { return height_; }
  double getResolution() const { return resolution_; }
  double getOriginX() const { return origin_x_; }
  double getOriginY() const { return origin_y_; }
  long long getVersion() const { return version_; }
  const std::vector<uint8_t>& getData() const { return grid_data_; }

  void setLethalThreshold(int t) { lethal_threshold_ = t; }
  void setUnknownCostFree(bool v) { unknown_cost_free_ = v; }
  void setUnknownValue(uint8_t v) { unknown_value_ = v; }

  void setVehicleSpec(const VehicleSpec& spec) {
    std::lock_guard<std::recursive_mutex> lock(map_mutex_);
    vehicle_spec_ = spec;
    has_vehicle_spec_ = true;
  }

  bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
  void gridToWorld(int gx, int gy, double& wx, double& wy) const;

  bool isInbounds(double x, double y) const;
  bool isInside(int gx, int gy) const;
  bool isObstacle(int gx, int gy) const;
  bool checkCollisionPoint(double x, double y) const;
  bool checkCollision(double x, double y, double theta) const;
  double getCollisionSafetyMargin(double x, double y, double theta) const;
  void setCell(int gx, int gy, uint8_t value);
  void setCells(const std::vector<std::pair<int, int>>& indices, const std::vector<uint8_t>& values);

  void updateMap(int width, int height, double res, double ox, double oy,
                 const std::vector<int8_t>& data);
  void setObstacle(int gx, int gy, bool active);

private:
  bool isInsideInternal(int gx, int gy) const { return gx >= 0 && gx < width_ && gy >= 0 && gy < height_; }
  bool isObstacleInternal(int gx, int gy) const;

  void setMeta(int width, int height, double res, double ox, double oy);
  void allocate(uint8_t fill_value);
  void bumpVersion() { version_++; }

  inline size_t getIndex(int x, int y) const {
    return static_cast<size_t>(y) * width_ + static_cast<size_t>(x);
  }

  mutable std::recursive_mutex map_mutex_;
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.05;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  long long version_ = 0;

  std::vector<uint8_t> grid_data_;
  std::vector<double> distance_map_;
  int lethal_threshold_ = 50;
  bool unknown_cost_free_ = true;
  uint8_t unknown_value_ = 255;

  VehicleSpec vehicle_spec_;
  bool has_vehicle_spec_ = false;

  GlobalCostmapConfig config_;

  void computeEDT();
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_GLOBAL_MAP_H
