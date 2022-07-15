#pragma once

#include "test_common/ChassisData.h"

namespace mujianhua {
namespace common {

class VehicleStateProvider {
  public:
    VehicleStateProvider() = default;

    bool Update(const test_common::ChassisData *chassis_data);

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
    const test_common::ChassisData *data_;
};

} // namespace common
} // namespace mujianhua