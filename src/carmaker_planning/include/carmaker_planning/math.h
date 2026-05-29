/**
 * @file math.h
 * @brief Unified math utility functions for carmaker_planning.
 */
#ifndef CARMAKER_PLANNING_MATH_H
#define CARMAKER_PLANNING_MATH_H

#include <cmath>
#include <algorithm>

namespace carmaker_planning {

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

inline double wrap_to_pi(double angle) {
  double a = std::fmod(angle + PI, TWO_PI);
  if (a < 0.0) a += TWO_PI;
  return a - PI;
}

inline double wrap_to_2pi(double angle) {
  double a = std::fmod(angle, TWO_PI);
  if (a < 0.0) a += TWO_PI;
  return a;
}

inline double deg2rad(double deg) { return deg * PI / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / PI; }
inline double dist(double x1, double y1, double x2, double y2) { return std::hypot(x1 - x2, y1 - y2); }
inline double dist(double dx, double dy) { return std::hypot(dx, dy); }

inline double mengerCurvature(double x1, double y1, double x2, double y2, double x3, double y3) {
  const double area2 = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
  const double a = dist(x1, y1, x2, y2);
  const double b = dist(x2, y2, x3, y3);
  const double c = dist(x1, y1, x3, y3);
  const double denom = a * b * c;
  if (denom > 1e-9) return 2.0 * area2 / denom;
  return 0.0;
}

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_MATH_H
