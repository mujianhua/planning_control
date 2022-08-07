#pragma once

#include "config/planning_config.h"

namespace mujianhua {
namespace planning {
namespace common {

class DependencyInjector {
  public:
    DependencyInjector() = default;
    ~DependencyInjector() = default;

    PlanningConfig *planning_config() { return &planning_config_; }

  private:
    PlanningConfig planning_config_;
};

} // namespace common
} // namespace planning
} // namespace mujianhua