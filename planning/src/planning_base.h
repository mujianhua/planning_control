#pragma once

#include "common/planning_config.h"
#include "planner/planner.h"
#include "planning/discretized_trajectory.h"
#include "planning/frame.h"
#include "planning/local_view.h"

namespace planning {

class PlanningBase {
 public:
  PlanningBase() = delete;

  virtual ~PlanningBase() = default;

  explicit PlanningBase(const PlanningConfig &config) : config_(config) {}

  virtual void RunOnce(const LocalView &local_view,
                       DiscretizedTrajectory *const adc_trajectory) = 0;

  Frame *frame() const { return frame_.get(); }

 protected:
  PlanningConfig config_;

  LocalView local_view_;

  std::unique_ptr<Planner> planner_;
  std::unique_ptr<Frame> frame_;
};

}  // namespace planning
