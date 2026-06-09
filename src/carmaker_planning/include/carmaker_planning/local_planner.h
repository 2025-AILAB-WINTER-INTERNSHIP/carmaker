/**
 * @file local_planner.h
 * @brief Local planner: cusp-based segment manager and quintic polynomial path fitter.
 *
 * Architecture (RULE §1 — Pure C++ Core):
 *  - No ROS headers. Results returned via structs, not exceptions.
 *  - SegmentManager  : splits a global Path at direction-change (cusp) boundaries,
 *                      stores the active segment index, and advances on request.
 *  - QuinticPathFitter: fits 5th-order polynomials x(s), y(s) from current ego state
 *                      to the active segment endpoint as a raw geometric path.
 *                      Final sampling, heading, curvature, and timing are filled by
 *                      the common post-processing helpers.
 */
#ifndef CARMAKER_PLANNING_LOCAL_PLANNER_H
#define CARMAKER_PLANNING_LOCAL_PLANNER_H

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "carmaker_planning/types.h"
#include "carmaker_planning/post_processing.h"

namespace carmaker_planning {

// ── SegmentManager ────────────────────────────────────────────────────────────

/**
 * @brief Decomposes a global path into cusp-delimited segments and tracks
 *        the active segment index. Arrival decisions live in TrajectoryStateMachine.
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

  /// Advance to the next segment. Returns true if all segments are completed.
  bool advanceToNextSegment();

private:
  static std::vector<Segment> split(const Path& path);

  Path                 global_path_;
  std::vector<Segment> segments_;
  size_t               active_idx_ = 0;
  bool                 finished_   = false;
};

// ── QuinticPathFitter ─────────────────────────────────────────────────────────

/**
 * @brief Generates a raw local path by fitting quintic (5th-order)
 *        polynomials x(s) and y(s) between ego and target.
 *
 * Boundary conditions (6 per axis, using fixed-size Eigen per RULE §3):
 *   s=0 : x, y,  dx/ds=cosθ₀,  dy/ds=sinθ₀,  x''=-κ₀sinθ₀,  y''=κ₀cosθ₀
 *   s=L : x, y,  dx/ds=cosθ_f, dy/ds=sinθ_f,  x''=-κ_f sinθ_f, y''=κ_f cosθ_f
 *
 * Sampling: points are spaced in the polynomial parameter before the path is
 * resampled by the common post-processing helper. The last point always matches
 * the target endpoint exactly.
 *
 * Returns empty path on degenerate input (L < 1e-3 m).
 */
class QuinticPathFitter {
public:
  QuinticPathFitter() = default;

  Path fit(const State& ego, double start_kappa,
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
  static Path makeShortDistanceFallback(const State& ego, const PathPoint& target, double length);
};

// ── LocalTrajectoryPlanner ───────────────────────────────────────────────────

class LocalTrajectoryPlanner {
public:
  struct PlanRequest {
    State ego;
    double start_kappa = 0.0;
    PathPoint target;
    double start_vel = 0.0;
  };

  LocalTrajectoryPlanner() = default;

  void configure(const PostProcessConfig& post_process_config,
                 double wheelbase,
                 double min_turning_radius);

  LocalPlanningResult planToEndpoint(const PlanRequest& request) const;

private:
  bool validateRequest(const PlanRequest& request,
                       LocalPlanningResult& result) const;

  PostProcessConfig post_process_config_;
  std::unique_ptr<PostProcessor> post_processor_;
  bool configured_ = false;
  double wheelbase_ = 2.97;
  double min_turning_radius_ = 5.2;
  QuinticPathFitter fitter_;
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_LOCAL_PLANNER_H
