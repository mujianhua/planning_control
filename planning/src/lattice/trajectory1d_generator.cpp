/**
 * @file trajectory1d_generator.cpp
 */

#include "lattice/trajectory1d_generator.h"

namespace planning {

Trajectory1dGenerator::Trajectory1dGenerator(
    const std::array<double, 3> &lon_init_state,
    const std::array<double, 3> &lat_init_state) {}

void Trajectory1dGenerator::GenerateTrajectoryBundles(
    std::vector<std::shared_ptr<Curve1d>> *ptr_lon_trajectory_bundle,
    std::vector<std::shared_ptr<Curve1d>> *ptr_lat_trajectory_bundle) {}

void Trajectory1dGenerator::GenerateLateralTrajectoryBundle(
    std::vector<std::shared_ptr<Curve1d>> *ptr_lat_trajectory_bundle) const {}

}  // namespace planning
