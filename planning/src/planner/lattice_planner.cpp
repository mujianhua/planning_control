
#include "lattice_planner.h"

#include <array>
#include <memory>

#include "../common/data_struct.h"
#include "../lattice/path_time_graph.h"
#include "../math/cartesian_frenet_conversion.h"

namespace planning {

using planning::math::CartesianFrenetConverter;

bool LatticePlanner::Plan(const TrajectoryPoint &planning_init_point,
                          Frame *frame, DiscretizedTrajectory &result) {
  TrajectoryPoint match_point = frame->reference_line().GetProjection(
      {planning_init_point.x, planning_init_point.y});

  std::array<double, 3> init_s{};
  std::array<double, 3> init_d{};
  TrajectoryPoint cartesian_point;
  cartesian_point.x = frame->vehicle_state().x;
  cartesian_point.y = frame->vehicle_state().y;
  cartesian_point.theta = frame->vehicle_state().theta;
  cartesian_point.kappa = match_point.kappa;
  cartesian_point.velocity = frame->vehicle_state().v;
  cartesian_point.a = frame->vehicle_state().a;
  ComputeInitFrenetState(match_point, cartesian_point, &init_s, &init_d);

  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
      frame->obstacles(), &frame->reference_line(), init_s[0], init_s[0], 0.0,
      0.0, init_d);

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
