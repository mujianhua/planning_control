#pragma once

namespace mujianhua {
namespace planning {

class VehicleState2 {
  public:
    VehicleState2() = default;
    VehicleState2(double x, double y, double heading);

    void Update(const double &x, const double &y, const double &heading);

    const double &x() const;

    const double &y() const;

    const double &heading() const;

    const double &lateral_error() const;

    const double &heading_error() const;

    double &x();

    double &y();

    double &heading();

    void SetInitialError(double lateral_error, double heading_error);

  private:
    double x_{}, y_{}, heading_{};
    double lateral_error_{}, heading_error_{};
};

} // namespace planning
} // namespace mujianhua