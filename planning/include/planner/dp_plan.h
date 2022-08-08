#pragma once

#include "common/discretized_trajectory.h"
#include "common/frame.h"
#include "config/planning_config.h"
#include "constraint_checker/collision_checker.h"

namespace mujianhua {
namespace planning {

namespace {
constexpr int NT = 5;
constexpr int NS = 7;
constexpr int NL = 10;
} // namespace

class DPPlan {
  public:
    DPPlan(const PlanningConfig &config);

    bool Plan(const State &start_state, const Frame *frame,
              DiscretizedTrajectory &result);

  private:
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

    struct StateIndex {
        int t = -1, s = -1, l = -1;

        StateIndex() = default;

        StateIndex(int tt, int ss, int ll) : t(tt), s(ss), l(ll) {}
    };

    struct StartState {
        double start_s = 0;
        double start_l = 0;
        double start_theta = 0;
    };

    double GetLateralOffset(double s, int l_ind);

    double GetCollisionCost(StateIndex parent_ind, StateIndex cur_ind);

    std::pair<double, double> GetCost(StateIndex parent_ind,
                                      StateIndex cur_ind);

    std::vector<math::Vec2d> InterpolateLinearly(double parent_s,
                                                 int parent_l_ind,
                                                 int cur_s_ind, int cur_l_ind);

  private:
    const PlanningConfig config_;
    const Frame *frame_;

    CollisionChecker *collision_checker_;

    double safe_margin_;

    int nseg_;
    double unit_time_;
    std::array<double, NT> time_;
    std::array<double, NS> station_;
    std::array<double, NL - 1> lateral_;

    StartState state_;
    StateCell state_space_[NT][NS][NL];
};

} // namespace planning
} // namespace mujianhua
