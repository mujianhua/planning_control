/**
 * @file data_struct.h
 * @brief provide data structure for planning.
 */

#pragma once

#include <vector>

namespace planning {

struct VehicleState {
  explicit VehicleState(double xx = 0.0, double yy = 0.0, double ttheta = 0.0,
                        double vv = 0.0, double pphi = 0.0, double aa = 0.0,
                        double oomega = 0.0)
      : x(xx), y(yy), theta(ttheta), v(vv), phi(pphi), a(aa), omega(oomega) {}
  double x, y, theta, v, phi, a, omega;
};

struct PathPoint {
  explicit PathPoint(double ss = 0.0, double xx = 0.0, double yy = 0.0,
                     double ttheta = 0.0, double kkappa = 0.0,
                     double ddkappa = 0.0)
      : s(ss), x(xx), y(yy), theta(ttheta), kappa(kkappa), dkappa(ddkappa) {}

  double s = 0.0;

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double dkappa = 0.0;
};

struct TrajectoryPoint {
  explicit TrajectoryPoint(double ss = 0.0, double xx = 0.0, double yy = 0.0,
                           double ttheta = 0.0, double kkappa = 0.0,
                           double ddkappa = 0.0)
      : s(ss), x(xx), y(yy), theta(ttheta), kappa(kkappa), dkappa(ddkappa) {}

  double s = 0.0;

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double dkappa = 0.0;

  double velocity = 0.0;
  double a = 0.0;

  double left_bound = 0.0;
  double right_bound = 0.0;
};

struct SLBoundary {
  explicit SLBoundary(double start_ss, double end_ss, double start_ll,
                      double end_ll)
      : start_s(start_ss), end_s(end_ss), start_l(start_ll), end_l(end_ll) {}

  double start_s;
  double end_s;
  double start_l;
  double end_l;
};

using Trajectory = std::vector<TrajectoryPoint>;

}  // namespace planning
