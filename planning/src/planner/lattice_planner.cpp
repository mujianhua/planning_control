
#include "planner/lattice_planner.h"
#include <array>
#include "math/cartesian_frenet_conversion.h"
#include "planning/data_struct.h"

namespace planning {

using planning::math::CartesianFrenetConverter;

bool LatticePlanner::Plan(const VehicleState &start_state, Frame *frame,
                          DiscretizedTrajectory &result) {
  TrajectoryPoint match_point =
      frame->reference_line().GetProjection({start_state.x, start_state.y});

  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  TrajectoryPoint cartesian_point;
  cartesian_point.x = start_state.x;
  // ComputeInitFrenetState(match_point, const TrajectoryPoint &cartesian_point,
  //                        &init_s, &init_d);

  return true;
}

void LatticePlanner::ComputeInitFrenetState(
    const TrajectoryPoint &match_point, const TrajectoryPoint &cartesian_point,
    std::array<double, 3> *ptr_s, std::array<double, 3> *ptr_d) {
  CartesianFrenetConverter::cartesian_to_frenet(
      match_point.s, match_point.x, match_point.y, match_point.theta,
      match_point.kappa, match_point.dkappa, cartesian_point.x,
      cartesian_point.y, cartesian_point.velocity, 0.0, cartesian_point.theta,
      cartesian_point.kappa, ptr_s, ptr_d);
}

}  // namespace planning
