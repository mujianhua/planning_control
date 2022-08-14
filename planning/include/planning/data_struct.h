#pragma once
#include <vector>

namespace planning {

struct VehicleState {
  VehicleState(double xx = 0.0, double yy = 0.0, double ttheta = 0.0,
               double vv = 0.0, double pphi = 0.0, double aa = 0.0,
               double oomega = 0.0)
      : x(xx), y(yy), theta(ttheta), v(vv), phi(pphi), a(aa), omega(oomega) {}
  double x, y, theta, v, phi, a, omega;
};

struct PathPoint {
  double s = 0.0;

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double dkappa = 0.0;
};

struct TrajectoryPoint {
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

using Trajectory = std::vector<TrajectoryPoint>;

}  // namespace planning
