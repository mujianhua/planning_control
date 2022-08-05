#pragma once

#include <memory>
#include <string>
#include <utility>
#include "common/dependency_injector.h"

namespace mujianhua {
namespace planning {

class Planner {
  public:
    Planner() = delete;

    explicit Planner(std::shared_ptr<common::DependencyInjector> injector)
        : injector_(std::move(injector)) {}

    virtual std::string Name() = 0;

  private:
    std::shared_ptr<common::DependencyInjector> injector_;
};

} // namespace planning
} // namespace mujianhua