#pragma once

#include <memory>
#include <string>
#include <utility>
#include "common/dependency_injector.h"
#include "common/discretized_trajectory.h"
#include "common/frame.h"
#include "common/vehicle_state.h"
#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

class Planner {
  public:
    Planner() = delete;

    explicit Planner(std::shared_ptr<DependencyInjector> injector)
        : injector_(std::move(injector)) {}

    virtual bool Init(const PlanningConfig &config) = 0;

    virtual std::string Name() = 0;

    virtual bool Plan(const State &start_state, Frame *frame,
                      DiscretizedTrajectory *ptr_computed_trajectory) = 0;

  protected:
    PlanningConfig config_;
    std::shared_ptr<DependencyInjector> injector_;
};

} // namespace planning
} // namespace mujianhua