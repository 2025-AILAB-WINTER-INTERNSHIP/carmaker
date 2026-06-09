/**
 * @file local_planner.cpp
 * @brief Implementation of SegmentManager and QuinticPathFitter.
 *
 * RULE §1 — Pure C++ Core: No ROS headers included.
 * RULE §3 — Static Eigen Matrices: All matrix operations use fixed-size types.
 */

#include "carmaker_planning/local_planner.h"
#include "carmaker_planning/math.h"
#include <cmath>

namespace carmaker_planning {

// ── SegmentManager ────────────────────────────────────────────────────────────

void SegmentManager::setGlobalPath(const Path& path) {
  global_path_ = path;
  segments_    = split(path);
  active_idx_  = 0;
  finished_    = segments_.empty();
}

std::vector<SegmentManager::Segment> SegmentManager::split(const Path& path) {
  std::vector<Segment> segs;
  if (path.empty()) return segs;

  size_t start = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    if (path[i].direction != path[start].direction) {
      segs.push_back({start, i - 1, path[start].direction});
      start = i;
    }
  }
  segs.push_back({start, path.size() - 1, path[start].direction});
  return segs;
}

const SegmentManager::Segment* SegmentManager::activeSegment() const {
  if (finished_ || active_idx_ >= segments_.size()) return nullptr;
  return &segments_[active_idx_];
}

const PathPoint* SegmentManager::activeEndpoint() const {
  const Segment* seg = activeSegment();
  if (!seg) return nullptr;
  return &global_path_[seg->end_idx];
}

Path SegmentManager::activeSegmentPath() const {
  const Segment* seg = activeSegment();
  if (!seg) return {};
  return Path(global_path_.begin() + static_cast<std::ptrdiff_t>(seg->start_idx),
              global_path_.begin() + static_cast<std::ptrdiff_t>(seg->end_idx + 1));
}

bool SegmentManager::advanceToNextSegment() {
  if (finished_) return true;
  ++active_idx_;
  finished_ = (active_idx_ >= segments_.size());
  return finished_;
}

// ── QuinticPathFitter ─────────────────────────────────────────────────────────

Eigen::Matrix<double,6,1> QuinticPathFitter::solveAxis(
    double p0, double dp0, double ddp0,
    double pL, double dpL, double ddpL, double L) {
  // Boundary conditions:
  //   f(0)   = p0,   f'(0)   = dp0,   f''(0)   = ddp0
  //   f(L)   = pL,   f'(L)   = dpL,   f''(L)   = ddpL
  //
  // f(s) = a0 + a1*s + a2*s² + a3*s³ + a4*s⁴ + a5*s⁵
  //
  // s=0 conditions fix a0, a1, a2 directly:
  //   a0 = p0,  a1 = dp0,  a2 = ddp0/2
  //
  // s=L conditions give a 3×3 system for [a3, a4, a5].

  const double L2 = L*L, L3 = L2*L, L4 = L3*L, L5 = L4*L;
  const double a0 = p0, a1 = dp0, a2 = 0.5 * ddp0;

  // Residuals at s = L
  const double r0 = pL   - (a0 + a1*L + a2*L2);
  const double r1 = dpL  - (a1 + 2.0*a2*L);
  const double r2 = ddpL - (2.0*a2);

  // Fixed-size 3×3 system (RULE §3 — no dynamic MatrixXd)
  //  [L3    L4     L5  ] [a3]   [r0]
  //  [3L2   4L3    5L4 ] [a4] = [r1]
  //  [6L    12L2   20L3] [a5]   [r2]
  Eigen::Matrix3d A;
  A << L3,      L4,       L5,
       3.0*L2,  4.0*L3,   5.0*L4,
       6.0*L,   12.0*L2,  20.0*L3;

  const Eigen::Vector3d x = A.lu().solve(Eigen::Vector3d(r0, r1, r2));

  Eigen::Matrix<double,6,1> coeff;
  coeff << a0, a1, a2, x(0), x(1), x(2);
  return coeff;
}

// Evaluate polynomial at arc-length s using Horner's method (numerically stable).
double QuinticPathFitter::eval(const Eigen::Matrix<double,6,1>& c, double s) {
  return c(0) + s*(c(1) + s*(c(2) + s*(c(3) + s*(c(4) + s*c(5)))));
}

// First derivative: f'(s)
static inline double deriv1(const Eigen::Matrix<double,6,1>& c, double s) {
  return c(1) + s*(2.0*c(2) + s*(3.0*c(3) + s*(4.0*c(4) + s*5.0*c(5))));
}

// Second derivative: f''(s)
static inline double deriv2(const Eigen::Matrix<double,6,1>& c, double s) {
  return 2.0*c(2) + s*(6.0*c(3) + s*(12.0*c(4) + s*20.0*c(5)));
}

double QuinticPathFitter::curvature(const Eigen::Matrix<double,6,1>& cx,
                                    const Eigen::Matrix<double,6,1>& cy, double s) {
  const double xp  = deriv1(cx, s), yp  = deriv1(cy, s);
  const double xpp = deriv2(cx, s), ypp = deriv2(cy, s);
  // κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
  const double speed_sq = xp*xp + yp*yp;
  const double denom    = speed_sq * std::sqrt(speed_sq);  // avoids std::pow
  return (denom > 1e-9) ? (xp*ypp - yp*xpp) / denom : 0.0;
}

Path QuinticPathFitter::fit(const State& ego, double ego_kappa,
                             const PathPoint& target, double resolution) const {
  const double L = dist(ego.x, ego.y, target.x, target.y);
  if (L < 1e-3) {
    return makeShortDistanceFallback(ego, target, L);
  }

  const int direction = target.direction < 0 ? -1 : 1;
  const double start_tangent = (direction == 1) ? ego.theta : wrap_to_pi(ego.theta + PI);
  const double target_tangent = (direction == 1) ? target.theta : wrap_to_pi(target.theta + PI);
  const double cos0 = std::cos(start_tangent), sin0 = std::sin(start_tangent);
  const double cosF = std::cos(target_tangent), sinF = std::sin(target_tangent);

  // Second derivatives from curvature (unit-speed parametrization):
  //   x'' = -κ sinθ,   y'' = κ cosθ
  const auto cx = solveAxis(ego.x,   cos0, -ego_kappa    * sin0,
                             target.x, cosF, -target.kappa * sinF, L);
  const auto cy = solveAxis(ego.y,   sin0,  ego_kappa    * cos0,
                             target.y, sinF,  target.kappa * cosF, L);

  // Uniform arc-length sampling:
  //   n_steps = ceil(L / resolution) → each step is exactly ds = L/n_steps
  //   The last sample (i == n_steps) always equals s = L (endpoint).
  const int    n_steps  = std::max(1, static_cast<int>(std::ceil(L / resolution)));
  const double ds       = L / n_steps;

  Path local_path;
  local_path.reserve(static_cast<size_t>(n_steps) + 1);

  for (int i = 0; i <= n_steps; ++i) {
    const double s = i * ds;
    PathPoint pt;
    pt.x         = eval(cx, s);
    pt.y         = eval(cy, s);
    pt.s         = s;
    pt.direction = direction;

    const double xp = deriv1(cx, s);
    const double yp = deriv1(cy, s);
    pt.theta = (direction == 1) ? std::atan2(yp, xp)
                                : wrap_to_pi(std::atan2(yp, xp) + PI);
    pt.kappa = curvature(cx, cy, s);
    local_path.push_back(pt);
  }
  return local_path;
}

Path QuinticPathFitter::makeShortDistanceFallback(
    const State& ego,
    const PathPoint& target,
    double length) {
  const int direction = target.direction < 0 ? -1 : 1;

  Path path;
  path.reserve(2);

  PathPoint start;
  start.x = ego.x;
  start.y = ego.y;
  start.theta = ego.theta;
  start.kappa = 0.0;
  start.s = 0.0;
  start.direction = direction;
  path.push_back(start);

  PathPoint end;
  end.x = target.x;
  end.y = target.y;
  end.theta = ego.theta;
  end.kappa = 0.0;
  end.s = std::max(0.0, length);
  end.direction = direction;
  path.push_back(end);

  return path;
}

} // namespace carmaker_planning
