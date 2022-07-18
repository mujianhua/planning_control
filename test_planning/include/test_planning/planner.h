/**
 * @file
 * @brief
 */

#pragma once

#include <memory>
#include <string>
#include "common_me/TrajectoryPoint.h"
#include "common_me/status.h"
#include "test_planning/dependency_injector.h"
#include "test_planning/planning_config.h"

namespace mujianhua {
namespace planning {

class Planner {
  public:
    /**
     * @brief Constructor
     */
    Planner() = delete;

    explicit Planner(const std::shared_ptr<DependencyInjector> &injector)
        : injector_(injector) {}

    virtual ~Planner() = default;

    virtual std::string Name() = 0;

    virtual common::Status Init(const PlanningConfig &planning_config) = 0;

    virtual bool
    Plan(const common_me::TrajectoryPoint &planning_init_point) = 0;

  private:
    std::shared_ptr<DependencyInjector> injector_;
};

class PlannerWithReferenceLine : public Planner {
  public:
    PlannerWithReferenceLine() = delete;

    explicit PlannerWithReferenceLine(
        const std::shared_ptr<DependencyInjector> &injector)
        : Planner(injector) {}

    virtual bool
    PlanOnReferenceLine(const common_me::TrajectoryPoint &planning_init_point) {
    }
};

} // namespace planning
} // namespace mujianhua