#pragma once

#include <memory>
#include <string>
#include "planner/planner.h"

namespace mujianhua {
namespace planning {

class CartesianPlanner : public Planner {
  public:
    CartesianPlanner() = delete;

    explicit CartesianPlanner(
        const std::shared_ptr<common::DependencyInjector> &injector);

    std::string Name() override { return "Cartesian Planner"; };

    bool Plan(const State &start_state,
              common::Frame *frame) override;

  private:
    std::shared_ptr<common::DependencyInjector> injector_;
};

} // namespace planning
} // namespace mujianhua