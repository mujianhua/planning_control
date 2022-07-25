#pragma once

namespace mujianhua {
namespace planning {

class ReferencePoint {
  public:
    ReferencePoint(double x, double y, double s, double theta, double kappa);

  private:
    double x_{}, y_{}, s_{}, theta_{}, kappa_{};
};

} // namespace planning
} // namespace mujianhua