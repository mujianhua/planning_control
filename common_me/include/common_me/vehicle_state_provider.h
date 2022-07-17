#pragma once

#include "common_me/ChassisData.h"

namespace mujianhua {
namespace common {

class VehicleStateProvider {
  public:
    VehicleStateProvider() = default;

    bool Update(const common_me::ChassisData *chassis_data);

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
    const common_me::ChassisData *data_;
};

} // namespace common
} // namespace mujianhua