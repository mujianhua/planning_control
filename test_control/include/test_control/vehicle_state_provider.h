#pragma once
#include "test_control/chassis_data.h"
namespace mujianhua {
namespace control {

class VehicleStateProvider {
  public:
    VehicleStateProvider() = default;

    bool Update(const test_control::chassis_data *chassis_data);

    double x() const;

    double y() const;

    double theta() const;

    double roll() const;

    double pitch() const;

    double vx() const;

    double vy() const;

    double yawrate() const;

    double rollrate() const;

  private:
    const test_control::chassis_data *data_;
};

} // namespace control
} // namespace mujianhua