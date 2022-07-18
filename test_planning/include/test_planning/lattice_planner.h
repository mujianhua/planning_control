/**
 * @file
 * @brief
 */
#pragma once

#include "test_planning/planner.h"

namespace mujianhua {
namespace planning {

class LatticePlanner : public PlannerWithReferenceLine {
  public:
    LatticePlanner() = delete;

    explicit LatticePlanner(const std::shared_ptr<DependencyInjector> &injector)
        : PlannerWithReferenceLine(injector) {}

    virtual ~LatticePlanner() = default;

    std::string Name() { return "LATTICE"; }

    common::Status Init(const PlanningConfig &planning_config) override;

    bool Plan(const common_me::TrajectoryPoint &planning_init_point);

  private:
};

} // namespace planning
} // namespace mujianhua