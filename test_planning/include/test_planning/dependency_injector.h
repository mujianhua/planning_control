
#pragma once

#include "test_common/vehicle_state_provider.h"

namespace mujianhua {
namespace planning {

class DependencyInjector {
  public:
    DependencyInjector() = default;
    ~DependencyInjector() = default;

    mujianhua::common::VehicleStateProvider *vehicle_state() {
        return &vehicle_state_;
    }

  private:
    mujianhua::common::VehicleStateProvider vehicle_state_;
};

} // namespace planning
} // namespace mujianhua