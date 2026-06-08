/**
 * @file local_planner.h
 * @brief Local planner: cusp-based segment manager and quintic polynomial path fitter.
 *
 * Architecture (RULE §1 — Pure C++ Core):
 *  - No ROS headers. Results returned via structs, not exceptions.
 *  - SegmentManager  : splits a global Path at direction-change (cusp) boundaries,
 *                      tracks the active segment, and detects arrival at each endpoint.
 *  - QuinticPathFitter: fits 5th-order polynomials x(s), y(s) from current ego state
 *                      to the active segment endpoint, satisfying 6 boundary conditions
 *                      per axis (position, heading, curvature) at both ends.
 *
 * Sampling guarantee:
 *  - Output points are uniformly spaced at exactly `resolution` [m] in arc-length,
 *    except the final point which always coincides with the target endpoint.
 */
#ifndef CARMAKER_PLANNING_LOCAL_PLANNER_H
#define CARMAKER_PLANNING_LOCAL_PLANNER_H

#include <vector>
#include <string>
#include <Eigen/Dense>
#include "carmaker_planning/types.h"

namespace carmaker_planning {

// ── SegmentManager ────────────────────────────────────────────────────────────

/**
 * @brief Decomposes a global path into cusp-delimited segments and tracks
 *        which segment the ego vehicle should currently follow.
 *
 * Thread-safety: NOT thread-safe. Callers must protect with a mutex.
 */
class SegmentManager {
public:
  struct Segment {
    size_t start_idx;  ///< Index into the global path (inclusive)
    size_t end_idx;    ///< Index into the global path (inclusive)
    int    direction;  ///< +1: forward, -1: reverse
  };

  SegmentManager() = default;

  /// Replace the stored global path and recompute segments. Resets active index to 0.
  void setGlobalPath(const Path& path);

  bool hasPath()    const { return !global_path_.empty(); }
  bool isFinished() const { return finished_; }
  size_t activeSegmentIndex() const { return active_idx_; }
  size_t segmentCount()       const { return segments_.size(); }

  /// Returns the current active segment, or nullptr when finished.
  const Segment* activeSegment() const;

  /// Returns the endpoint PathPoint of the active segment.
  const PathPoint* activeEndpoint() const;

  /// Returns a copy of path points belonging to the active segment.
  Path activeSegmentPath() const;

  const Path& globalPath() const { return global_path_; }

  /**
   * @brief Check arrival at the active segment endpoint and advance if arrived.
   * @return true if all segments completed (final goal reached).
   */
  bool updateArrival(const State& ego, double xy_tol, double yaw_tol, double vel_tol);

private:
  static std::vector<Segment> split(const Path& path);

  Path                 global_path_;
  std::vector<Segment> segments_;
  size_t               active_idx_ = 0;
  bool                 finished_   = false;
};

// ── QuinticPathFitter ─────────────────────────────────────────────────────────

/**
 * @brief Generates a uniform-spacing local path by fitting quintic (5th-order)
 *        polynomials x(s) and y(s) between ego and target.
 *
 * Boundary conditions (6 per axis, using fixed-size Eigen per RULE §3):
 *   s=0 : x, y,  dx/ds=cosθ₀,  dy/ds=sinθ₀,  x''=-κ₀sinθ₀,  y''=κ₀cosθ₀
 *   s=L : x, y,  dx/ds=cosθ_f, dy/ds=sinθ_f,  x''=-κ_f sinθ_f, y''=κ_f cosθ_f
 *
 * Sampling: points are spaced at exactly `resolution` [m] in arc-length parameter s.
 * The last point always matches the target endpoint exactly.
 *
 * Returns empty path on degenerate input (L < 1e-3 m).
 */
class QuinticPathFitter {
public:
  QuinticPathFitter() = default;

  Path fit(const State& ego, double ego_kappa,
           const PathPoint& target, double resolution) const;

private:
  /// Solve for polynomial coefficients [a0..a5] given boundary conditions.
  /// Uses fixed-size 3x3 Eigen matrix (RULE §3 — Static Eigen Matrices).
  static Eigen::Matrix<double,6,1> solveAxis(
      double p0, double dp0, double ddp0,
      double pL, double dpL, double ddpL, double L);

  static double eval(const Eigen::Matrix<double,6,1>& c, double s);

  static double curvature(const Eigen::Matrix<double,6,1>& cx,
                          const Eigen::Matrix<double,6,1>& cy, double s);
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_LOCAL_PLANNER_H
