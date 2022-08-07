#pragma once

#include "common/discretized_trajectory.h"
#include "common/frame.h"
#include "config/planning_config.h"

namespace mujianhua {
namespace planning {

class DPPlan {
  public:
    DPPlan(const PlanningConfig &config);

    bool Plan(const common::State &start_state, const common::Frame *frame,
              common::DiscretizedTrajectory &result);

  private:
    const PlanningConfig config_;
    const common::Frame *frame_;

    struct StateCell {
        double cost = std::numeric_limits<double>::max();
        double current_s = std::numeric_limits<double>::min();
        int parent_s_ind = -1;
        int parent_l_ind = -1;

        StateCell() = default;

        StateCell(double cost, double cur_s, int parent_s_ind, int parent_l_ind)
            : cost(cost), current_s(cur_s), parent_s_ind(parent_s_ind),
              parent_l_ind(parent_l_ind) {}
    };
};

} // namespace planning
} // namespace mujianhua
