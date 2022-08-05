#pragma once

#include <memory>
#include <utility>
#include "common/dependency_injector.h"
#include "planner/planner.h"

namespace mujianhua {
namespace planning {

class PlanningBase {
  public:
    PlanningBase() = delete;

    explicit PlanningBase(std::shared_ptr<common::DependencyInjector> injector)
        : injector_(std::move(injector)) {}

    virtual bool Init();

    virtual void RunOnce() = 0;

  protected:
    std::shared_ptr<common::DependencyInjector> injector_;
};

} // namespace planning
} // namespace mujianhua