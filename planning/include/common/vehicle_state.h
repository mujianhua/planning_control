#pragma once
namespace mujianhua {
namespace planning {
namespace common {

struct State {
    State(double xx = 0.0, double yy = 0.0, double ttheta = 0.0,
          double vv = 0.0, double pphi = 0.0, double aa = 0.0,
          double oomega = 0.0)
        : x(xx), y(yy), theta(ttheta), v(vv), phi(pphi), a(aa), omega(oomega) {}
    double x, y, theta, v, phi, a, omega;
};

} // namespace common
} // namespace planning
} // namespace mujianhua