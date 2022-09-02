/**
 * @file trajectory1d_generator.cpp
 */

#include "lattice/trajectory1d_generator.h"

#include "common/planning_gflags.h"
#include "lattice/lateral_osqp_optimizer.h"

namespace planning {

typedef std::array<double, 3> State;
typedef std::pair<State, double> Condition;
typedef std::vector<std::shared_ptr<Curve1d>> Trajectory1DBundle;

Trajectory1dGenerator::Trajectory1dGenerator(
    const std::array<double, 3> &lon_init_state,
    const std::array<double, 3> &lat_init_state,
    std::shared_ptr<PathTimeGraph> ptr_path_time_graph)
    : init_lon_state_(lon_init_state),
      init_lat_state_(lat_init_state),
      end_condition_sampler_(lon_init_state, lat_init_state,
                             ptr_path_time_graph),
      ptr_path_time_graph_(ptr_path_time_graph) {}

void Trajectory1dGenerator::GenerateTrajectoryBundles(
    Trajectory1DBundle *ptr_lon_trajectory_bundle,
    Trajectory1DBundle *ptr_lat_trajectory_bundle) {
  GenerateLongitudinalTrajectoryBundle(ptr_lon_trajectory_bundle);

  GenerateLateralTrajectoryBundle(ptr_lat_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateLateralTrajectoryBundle(
    std::vector<std::shared_ptr<Curve1d>> *ptr_lat_trajectory_bundle) const {}

void Trajectory1dGenerator::GenerateLongitudinalTrajectoryBundle(
    std::vector<std::shared_ptr<Curve1d>> *ptr_lon_trajectory_bundle) const {
  double s_min = init_lon_state_[0];
  double s_max = s_min + FLAGS_max_s_lateral_optimization;
  double delta_s = FLAGS_default_delta_s_lateral_optimization;

  auto lateral_bounds =
      ptr_path_time_graph_->GetLateralBounds(s_min, s_max, delta_s);
}

}  // namespace planning
