#pragma once

#include <memory>
#include <string>
#include <utility>
#include "common/dependency_injector.h"
#include "common/frame.h"
#include "common/vehicle_state.h"
#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

class Planner {
  public:
    Planner() = delete;

    explicit Planner(std::shared_ptr<common::DependencyInjector> injector)
        : injector_(std::move(injector)) {}

    virtual std::string Name() = 0;

    virtual bool Plan(const common::State &start_state,
                      common::Frame *frame) = 0;

  private:
    std::shared_ptr<common::DependencyInjector> injector_;
};

} // namespace planning
} // namespace mujianhua