/**
 * @file reeds_shepp.cpp
 * @brief Reeds-Shepp Curve Solver Implementation
 */

#include "carmaker_planning/reeds_shepp.h"

namespace carmaker_planning
{

namespace {

const double ZERO = 10.0 * std::numeric_limits<double>::epsilon();

// 18 path types defined by 5 segment elements.
const RSPartType reedsSheppPathType[18][5] = {
    {RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP},         // 0
    {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP},        // 1
    {RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP},       // 2
    {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP},       // 3
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 4
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 5
    {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},    // 6
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},   // 7
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 8
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 9
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},   // 10
    {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},    // 11
    {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},     // 12
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},     // 13
    {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},      // 14
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},    // 15
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT},  // 16
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT}   // 17
};

} // anonymous namespace


inline void ReedsSheppCurve::polar(double x, double y, double &r, double &theta)
{
  r = std::hypot(x, y);
  theta = std::atan2(y, x);
}

inline void ReedsSheppCurve::tauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega)
{
  double delta = wrap_to_pi(u - v);
  double A = std::sin(u) - std::sin(delta);
  double B = std::cos(u) - std::cos(delta) - 1.0;
  double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  tau = (t2 < 0.0) ? wrap_to_pi(t1 + PI) : wrap_to_pi(t1);
  omega = wrap_to_pi(tau - u + v - phi);
}

// Formula 8.1
inline bool ReedsSheppCurve::LpSpLp(double x, double y, double phi, double &t, double &u, double &v)
{
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
  if (t >= -ZERO)
  {
    v = wrap_to_pi(phi - t);
    if (v >= -ZERO)
    {
      return true;
    }
  }
  return false;
}

// Formula 8.2
inline bool ReedsSheppCurve::LpSpRp(double x, double y, double phi, double &t, double &u, double &v)
{
  double t1, u1;
  polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
  u1 = u1 * u1;
  if (u1 >= 4.0)
  {
    u = std::sqrt(u1 - 4.0);
    double theta = std::atan2(2.0, u);
    t = wrap_to_pi(t1 + theta);
    v = wrap_to_pi(t - phi);
    return t >= -ZERO && v >= -ZERO;
  }
  return false;
}

// Formula 8.3 / 8.4
inline bool ReedsSheppCurve::LpRmL(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  double u1, theta;
  polar(xi, eta, u1, theta);
  if (u1 <= 4.0)
  {
    u = -2.0 * std::asin(0.25 * u1);
    t = wrap_to_pi(theta + 0.5 * u + PI);
    v = wrap_to_pi(phi - t + u);
    return t >= -ZERO && u <= ZERO;
  }
  return false;
}

// Formula 8.7
inline bool ReedsSheppCurve::LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  if (rho <= 1.0)
  {
    u = std::acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    return t >= -ZERO && v <= ZERO;
  }
  return false;
}

// Formula 8.8
inline bool ReedsSheppCurve::LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho >= 0.0 && rho <= 1.0)
  {
    u = -std::acos(rho);
    if (u >= -0.5 * PI)
    {
      tauOmega(u, u, xi, eta, phi, t, v);
      return t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}

// Formula 8.9
inline bool ReedsSheppCurve::LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  double rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.0)
  {
    double r = std::sqrt(rho * rho - 4.0);
    u = 2.0 - r;
    t = wrap_to_pi(theta + std::atan2(r, -2.0));
    v = wrap_to_pi(phi - 0.5 * PI - t);
    return t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}

// Formula 8.10
inline bool ReedsSheppCurve::LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho, theta;
  polar(-eta, xi, rho, theta);
  if (rho >= 2.0)
  {
    t = theta;
    u = 2.0 - rho;
    v = wrap_to_pi(t + 0.5 * PI - phi);
    return t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}

// Formula 8.11
inline bool ReedsSheppCurve::LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.0)
  {
    u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u <= ZERO)
    {
      t = wrap_to_pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      v = wrap_to_pi(t - phi);
      return t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}

void ReedsSheppCurve::CSC(double x, double y, double phi, RSCandidate &path)
{
  double t, u, v, Lmin = path.get_length(), L;
  if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))
  {
    path = RSCandidate(reedsSheppPathType[14], t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // timeflip
  {
    path = RSCandidate(reedsSheppPathType[14], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path = RSCandidate(reedsSheppPathType[15], t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // timeflip + reflect
  {
    path = RSCandidate(reedsSheppPathType[15], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))
  {
    path = RSCandidate(reedsSheppPathType[12], t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // timeflip
  {
    path = RSCandidate(reedsSheppPathType[12], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path = RSCandidate(reedsSheppPathType[13], t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // timeflip + reflect
  {
    path = RSCandidate(reedsSheppPathType[13], -t, -u, -v);
  }
}

void ReedsSheppCurve::CCC(double x, double y, double phi, RSCandidate &path)
{
  double t, u, v, Lmin = path.get_length(), L;
  if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))
  {
    path = RSCandidate(reedsSheppPathType[0], t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // timeflip
  {
    path = RSCandidate(reedsSheppPathType[0], -t, -u, -v);
    Lmin = L;
  }
  if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path = RSCandidate(reedsSheppPathType[1], t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // timeflip + reflect
  {
    path = RSCandidate(reedsSheppPathType[1], -t, -u, -v);
    Lmin = L;
  }

  // backwards
  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))
  {
    path = RSCandidate(reedsSheppPathType[0], v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // timeflip
  {
    path = RSCandidate(reedsSheppPathType[0], -v, -u, -t);
    Lmin = L;
  }
  if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path = RSCandidate(reedsSheppPathType[1], v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, -yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // timeflip + reflect
  {
    path = RSCandidate(reedsSheppPathType[1], -v, -u, -t);
  }
}

void ReedsSheppCurve::CCCC(double x, double y, double phi, RSCandidate &path)
{
  double t, u, v, Lmin = path.get_length(), L;
  if (LpRupLumRm(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))
  {
    path = RSCandidate(reedsSheppPathType[2], t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))  // timeflip
  {
    path = RSCandidate(reedsSheppPathType[2], -t, -u, u, -v);
    Lmin = L;
  }
  if (LpRupLumRm(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))  // reflect
  {
    path = RSCandidate(reedsSheppPathType[3], t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))  // timeflip + reflect
  {
    path = RSCandidate(reedsSheppPathType[3], -t, -u, u, -v);
    Lmin = L;
  }

  if (LpRumLumRp(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))
  {
    path = RSCandidate(reedsSheppPathType[2], t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))  // timeflip
  {
    path = RSCandidate(reedsSheppPathType[2], -t, -u, -u, -v);
    Lmin = L;
  }
  if (LpRumLumRp(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))  // reflect
  {
    path = RSCandidate(reedsSheppPathType[3], t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))  // timeflip + reflect
  {
    path = RSCandidate(reedsSheppPathType[3], -t, -u, -u, -v);
  }
}

void ReedsSheppCurve::CCSC(double x, double y, double phi, RSCandidate &path)
{
  double t, u, v, Lmin = path.get_length(), L;
  constexpr double fixed_len = 0.5 * PI;

  if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))
  {
    path = RSCandidate(reedsSheppPathType[4], t, -fixed_len, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))  // timeflip
  {
    path = RSCandidate(reedsSheppPathType[4], -t, fixed_len, -u, -v);
    Lmin = L;
  }
  if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))  // reflect
  {
    path = RSCandidate(reedsSheppPathType[5], t, -fixed_len, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))  // timeflip + reflect
  {
    path = RSCandidate(reedsSheppPathType[5], -t, fixed_len, -u, -v);
    Lmin = L;
  }

  if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))
  {
    path = RSCandidate(reedsSheppPathType[8], t, -fixed_len, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))  // timeflip
  {
    path = RSCandidate(reedsSheppPathType[8], -t, fixed_len, -u, -v);
    Lmin = L;
  }
  if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))  // reflect
  {
    path = RSCandidate(reedsSheppPathType[9], t, -fixed_len, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))  // timeflip + reflect
  {
    path = RSCandidate(reedsSheppPathType[9], -t, fixed_len, -u, -v);
    Lmin = L;
  }

  // backwards
  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))
  {
    path = RSCandidate(reedsSheppPathType[6], v, u, -fixed_len, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))  // timeflip
  {
    path = RSCandidate(reedsSheppPathType[6], -v, -u, fixed_len, -t);
    Lmin = L;
  }
  if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))  // reflect
  {
    path = RSCandidate(reedsSheppPathType[7], v, u, -fixed_len, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, -yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))  // timeflip + reflect
  {
    path = RSCandidate(reedsSheppPathType[7], -v, -u, fixed_len, -t);
    Lmin = L;
  }

  if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))
  {
    path = RSCandidate(reedsSheppPathType[10], v, u, -fixed_len, t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))  // timeflip
  {
    path = RSCandidate(reedsSheppPathType[10], -v, -u, fixed_len, -t);
    Lmin = L;
  }
  if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))  // reflect
  {
    path = RSCandidate(reedsSheppPathType[11], v, u, -fixed_len, t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, -yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + fixed_len))
  {
    path = RSCandidate(reedsSheppPathType[11], -v, -u, fixed_len, -t);
  }
}

void ReedsSheppCurve::CCSCC(double x, double y, double phi, RSCandidate &path)
{
  double t, u, v, Lmin = path.get_length(), L;
  constexpr double fixed_len = 0.5 * PI;
  constexpr double total_fixed_len = PI;

  if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + total_fixed_len))
  {
    path = RSCandidate(reedsSheppPathType[16], t, -fixed_len, u, -fixed_len, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + total_fixed_len))  // timeflip
  {
    path = RSCandidate(reedsSheppPathType[16], -t, fixed_len, -u, fixed_len, -v);
    Lmin = L;
  }
  if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + total_fixed_len))  // reflect
  {
    path = RSCandidate(reedsSheppPathType[17], t, -fixed_len, u, -fixed_len, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v) + total_fixed_len))  // timeflip + reflect
  {
    path = RSCandidate(reedsSheppPathType[17], -t, fixed_len, -u, fixed_len, -v);
  }
}

RSCandidate ReedsSheppCurve::getPath(double x, double y, double phi)
{
  RSCandidate path;
  CSC(x, y, phi, path);
  CCC(x, y, phi, path);
  CCCC(x, y, phi, path);
  CCSC(x, y, phi, path);
  CCSCC(x, y, phi, path);
  return path;
}

void ReedsSheppCurve::toLocalFrame(const State& start, const State& goal,
                                   double rho, double& x, double& y, double& phi)
{
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double cos_theta = std::cos(start.theta);
  const double sin_theta = std::sin(start.theta);

  // Transform coordinates to start-centric frame, normalized by rho
  x = (dx * cos_theta + dy * sin_theta) / rho;
  y = (-dx * sin_theta + dy * cos_theta) / rho;
  phi = wrap_to_pi(goal.theta - start.theta);
}

double ReedsSheppCurve::getShortestLength(const State& start, const State& goal, double rho)
{
  RSPath path;
  if (calculatePath(start, goal, rho, path)) {
    return path.total_length;
  }
  return std::numeric_limits<double>::infinity();
}

bool ReedsSheppCurve::calculatePath(const State& start, const State& goal,
                                    double rho, RSPath& path)
{
  double x, y, phi;
  toLocalFrame(start, goal, rho, x, y, phi);

  RSCandidate best = getPath(x, y, phi);

  if (best.total_length == std::numeric_limits<double>::infinity()) {
    return false;
  }

  path.rho = rho;
  path.total_length = best.total_length * rho;
  path.seg_lengths.clear();
  path.seg_dir.clear();
  path.seg_steer.clear();

  for (int i = 0; i < 5; ++i) {
    if (best.type[i] == RS_NOP) {
      break;
    }
    double len = best.length[i];
    if (std::abs(len) < 1e-6) {
      continue;
    }
    path.seg_lengths.push_back(std::abs(len));
    path.seg_dir.push_back(len >= 0.0 ? 1 : -1);
    
    if (best.type[i] == RS_LEFT) {
      path.seg_steer.push_back(1);
    } else if (best.type[i] == RS_STRAIGHT) {
      path.seg_steer.push_back(0);
    } else if (best.type[i] == RS_RIGHT) {
      path.seg_steer.push_back(-1);
    }
  }

  // 실제 경로 타입 정보는 seg_steer / seg_dir / seg_lengths 배열로 완전히 전달된다.
  // RSPathType 열거형과 1:1 매핑이 복잡하므로 UNKNOWN으로 명시적으로 표기한다.
  path.type = RSPathType::UNKNOWN;
  return true;
}

void ReedsSheppCurve::sample(RSPath& path, const State& start, double step_size)
{
  path.points.clear();

  const double rho = path.rho;
  const double inv_rho = 1.0 / rho;

  State current_state = start;
  double current_s_m = 0.0;

  path.points.reserve(static_cast<size_t>(path.total_length / step_size) + 4);
  
  PathPoint pt_start(current_state.x, current_state.y, current_state.theta);
  pt_start.s = 0.0;
  pt_start.kappa = 0.0;
  if (!path.seg_steer.empty()) {
    pt_start.kappa = (path.seg_steer[0] == 0) ? 0.0 : static_cast<double>(path.seg_steer[0]) * inv_rho;
  }
  pt_start.direction = (!path.seg_dir.empty()) ? path.seg_dir[0] : 1;
  path.points.push_back(pt_start);

  auto propagate_step = [&](int dir, int steer, double ds_m) {
    if (steer == 0) { // Straight
      current_state.x += dir * ds_m * std::cos(current_state.theta);
      current_state.y += dir * ds_m * std::sin(current_state.theta);
    } else { // Turn
      const double d_theta = (dir * steer * ds_m) * inv_rho;
      const double theta_new = current_state.theta + d_theta;
      current_state.x += rho * steer * (std::sin(theta_new) - std::sin(current_state.theta));
      current_state.y -= rho * steer * (std::cos(theta_new) - std::cos(current_state.theta));
      current_state.theta = theta_new;
    }

    PathPoint pt(current_state.x, current_state.y, current_state.theta);
    pt.s = current_s_m;
    pt.kappa = (steer == 0) ? 0.0 : static_cast<double>(steer) * inv_rho;
    pt.direction = dir;
    path.points.push_back(pt);
  };

  // Loop through segments
  for (size_t i = 0; i < path.seg_lengths.size(); ++i) {
    double seg_len_m = path.seg_lengths[i] * rho;
    double seg_s = 0.0;
    while (seg_s + step_size < seg_len_m) {
      seg_s += step_size;
      current_s_m += step_size;
      propagate_step(path.seg_dir[i], path.seg_steer[i], step_size);
    }
    // Final small segment
    double remaining = seg_len_m - seg_s;
    if (remaining > 1e-4) {
      current_s_m += remaining;
      propagate_step(path.seg_dir[i], path.seg_steer[i], remaining);
    }
  }
}

} // namespace carmaker_planning
