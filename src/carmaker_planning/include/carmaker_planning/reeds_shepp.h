/**
 * @file reeds_shepp.h
 * @brief Reeds-Shepp Curve Library for Analytic Expansion with Reverse.
 */
#ifndef CARMAKER_PLANNING_REEDS_SHEPP_H
#define CARMAKER_PLANNING_REEDS_SHEPP_H

#include "carmaker_planning/types.h"

namespace carmaker_planning {

enum RSPartType {
  RS_NOP = 0,
  RS_LEFT = 1,
  RS_STRAIGHT = 2,
  RS_RIGHT = 3
};

struct RSCandidate {
  const RSPartType *type = nullptr;
  double length[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double total_length = std::numeric_limits<double>::infinity();

  RSCandidate() = default;
  RSCandidate(const RSPartType *t, double l0, double l1, double l2, double l3 = 0.0, double l4 = 0.0)
    : type(t)
  {
    length[0] = l0;
    length[1] = l1;
    length[2] = l2;
    length[3] = l3;
    length[4] = l4;
    total_length = std::abs(l0) + std::abs(l1) + std::abs(l2) + std::abs(l3) + std::abs(l4);
  }

  double get_length() const {
    return total_length;
  }
};

class ReedsSheppCurve {
public:
  static double getShortestLength(const State& start, const State& goal, double rho);
  static bool calculatePath(const State& start, const State& goal, double rho, RSPath& path);
  static void sample(RSPath& path, const State& start, double step_size);

private:
  // Coordinate transformations
  static void toLocalFrame(const State& start, const State& goal, double rho, double& x, double& y, double& phi);

  // Math helpers
  static void polar(double x, double y, double &r, double &theta);
  static void tauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega);

  // Reeds-Shepp Base Equations (1990 paper)
  static bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v);
  static bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v);
  static bool LpRmL(double x, double y, double phi, double &t, double &u, double &v);
  static bool LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v);
  static bool LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v);
  static bool LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v);
  static bool LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v);
  static bool LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v);

  // Route selectors (groups of formulas)
  static void CSC(double x, double y, double phi, RSCandidate &path);
  static void CCC(double x, double y, double phi, RSCandidate &path);
  static void CCCC(double x, double y, double phi, RSCandidate &path);
  static void CCSC(double x, double y, double phi, RSCandidate &path);
  static void CCSCC(double x, double y, double phi, RSCandidate &path);

  static RSCandidate getPath(double x, double y, double phi);
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_REEDS_SHEPP_H
