#pragma once

#include <memory>
#include <utility>
#include "common/dependency_injector.h"
#include "common/frame.h"
#include "common/local_view.h"
#include "config/planning_config.h"
#include "planner/planner.h"

namespace mujianhua {
namespace planning {

class PlanningBase {
  public:
    PlanningBase() = delete;

    explicit PlanningBase(std::shared_ptr<common::DependencyInjector> injector)
        : injector_(std::move(injector)) {}

    virtual bool Init(const PlanningConfig &config);

    virtual void
    RunOnce(const LocalView &local_view,
            common::DiscretizedTrajectory *const adc_trajectory) = 0;

  protected:
    PlanningConfig config_;

    size_t seq_num_ = 0;

    LocalView local_view_;

    std::shared_ptr<common::DependencyInjector> injector_;
    std::unique_ptr<common::Frame> frame_;
    std::unique_ptr<Planner> planner_;
};

} // namespace planning
} // namespace mujianhua