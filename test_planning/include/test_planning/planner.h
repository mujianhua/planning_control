/**
 * @file
 * @brief
 */

#pragma once

#include <memory>
#include <string>
#include "test_common/TrajectoryPoint.h"
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

    virtual bool Init(const PlanningConfig &planning_config) = 0;

    virtual bool
    Plan(const test_common::TrajectoryPoint &planning_init_point) = 0;

  private:
    std::shared_ptr<DependencyInjector> injector_;
};

class PlannerWithReferenceLine : public Planner {
  public:
    PlannerWithReferenceLine() = delete;

    explicit PlannerWithReferenceLine(
        const std::shared_ptr<DependencyInjector> &injector)
        : Planner(injector) {}

    virtual bool PlanOnReferenceLine(
        const test_common::TrajectoryPoint &planning_init_point) {}
};

} // namespace planning
} // namespace mujianhua