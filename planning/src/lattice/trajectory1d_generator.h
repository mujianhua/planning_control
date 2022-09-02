/**
 * @file trajectory1d_generator.h
 * @brief
 *
 */

#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "lattice/end_condition_sampler.h"
#include "lattice/lattice_trajectory1d.h"
#include "lattice/path_time_graph.h"
#include "math/curve1d/curve1d.h"
#include "math/curve1d/quartic_polynomial_curve1d.h"
#include "math/curve1d/quintic_polynomial_curve1d.h"

namespace planning {

class Trajectory1dGenerator {
 public:
  Trajectory1dGenerator(const std::array<double, 3> &lon_init_state,
                        const std::array<double, 3> &lat_init_state,
                        std::shared_ptr<PathTimeGraph> ptr_path_time_graph);

  virtual ~Trajectory1dGenerator() = default;

  void GenerateTrajectoryBundles(
      std::vector<std::shared_ptr<Curve1d>> *ptr_lon_trajectory_bundle,
      std::vector<std::shared_ptr<Curve1d>> *ptr_lat_trajectory_bundle);

  void GenerateLateralTrajectoryBundle(
      std::vector<std::shared_ptr<Curve1d>> *ptr_lat_trajectory_bundle) const;

 private:
  void GenerateSpeedProfilesForCruising() const;

  void GenerateLongitudinalTrajectoryBundle(
      std::vector<std::shared_ptr<Curve1d>> *ptr_lon_trajectory_bundle) const;

  template <size_t N>
  void GenerateTrajectory1DBundle(
      const std::array<double, 3> &init_state,
      const std::vector<std::pair<std::array<double, 3>, double>>
          &end_conditions,
      std::vector<std::shared_ptr<Curve1d>> *ptr_trajectory_bundle);

 private:
  std::array<double, 3> init_lon_state_;
  std::array<double, 3> init_lat_state_;

  EndConditionSampler end_condition_sampler_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
};

template <>
inline void Trajectory1dGenerator::GenerateTrajectory1DBundle<4>(
    const std::array<double, 3> &init_state,
    const std::vector<std::pair<std::array<double, 3>, double>> &end_conditions,
    std::vector<std::shared_ptr<Curve1d>> *ptr_trajectory_bundle) {
  CHECK_NOTNULL(ptr_trajectory_bundle);

  ptr_trajectory_bundle->reserve(ptr_trajectory_bundle->size() +
                                 end_conditions.size());
  for (const auto &end_condition : end_conditions) {
    auto ptr_trajectory = std::make_shared<LatticeTrajectory1d>(
        std::shared_ptr<Curve1d>(new QuarticPolynomialCurve1d(
            init_state, {end_condition.first[1], end_condition.first[2]},
            end_condition.second)));
    ptr_trajectory->set_target_velocity(end_condition.first[1]);
    ptr_trajectory->set_target_time(end_condition.second);
    ptr_trajectory_bundle->push_back(ptr_trajectory);
  }
}

template <>
inline void Trajectory1dGenerator::GenerateTrajectory1DBundle<5>(
    const std::array<double, 3> &init_state,
    const std::vector<std::pair<std::array<double, 3>, double>> &end_conditions,
    std::vector<std::shared_ptr<Curve1d>> *ptr_trajectory_bundle) {
  CHECK_NOTNULL(ptr_trajectory_bundle);

  ptr_trajectory_bundle->reserve(ptr_trajectory_bundle->size() +
                                 end_conditions.size());
  for (const auto &end_condition : end_conditions) {
    auto ptr_trajectory = std::make_shared<LatticeTrajectory1d>(
        std::shared_ptr<Curve1d>(new QuinticPolynomialCurve1d(
            init_state, end_condition.first, end_condition.second)));

    ptr_trajectory->set_target_position(end_condition.first[0]);
    ptr_trajectory->set_target_velocity(end_condition.first[1]);
    ptr_trajectory->set_target_time(end_condition.second);
    ptr_trajectory_bundle->push_back(ptr_trajectory);
  }
}

}  // namespace planning
