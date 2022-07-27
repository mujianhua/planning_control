#pragma once

namespace mujianhua {
namespace planning {

class ReferencePoint {
  public:
    ReferencePoint(double x = 0.0, double y = 0.0, double s = 0.0,
                   double theta = 0.0, double kappa = 0.0);

    const double &x() const;

    const double &y() const;

    const double &s() const;

    const double &theta() const;

    const double &kappa() const;

    double &x();

    double &y();

    double &s();

    double &theta();

    double &kappa();

  private:
    double x_{}, y_{}, s_{}, theta_{}, kappa_{};
};

} // namespace planning
} // namespace mujianhua