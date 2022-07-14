/**
 * @file
 * @brief
 */

#pragma once

#include "planning_config.h"
#include "test_common/TrajectoryPoint.h"
#include <string>

namespace mujianhua {
namespace planning {

class Planner {
  public:
    /**
     * @brief Constructor
     */
    Planner() = delete;

    virtual ~Planner() = default;

    virtual std::string Name() = 0;

    virtual bool Init(const PlanningConfig &planning_config) = 0;

    virtual bool
    Plan(const test_common::TrajectoryPoint &planning_init_point) = 0;

  private:
};

class PlannerWithReferenceLine : public Planner {
  public:
    PlannerWithReferenceLine() = delete;

    virtual bool PlanOnReferenceLine(
        const test_common::TrajectoryPoint &planning_init_point) {}
};

} // namespace planning
} // namespace mujianhua