
#pragma once

#include <cassert>
#include <utility>
#include <vector>

#include "../common/data_struct.h"
#include "../math/vec2d.h"

namespace planning {

using math::Vec2d;

/**
 * Discretized Trajectory
 */
class DiscretizedTrajectory {
 public:
  using DataType = std::vector<TrajectoryPoint>;

  DiscretizedTrajectory() = default;

  DiscretizedTrajectory(const DiscretizedTrajectory &rhs, size_t begin,
                        size_t end = -1);

  explicit DiscretizedTrajectory(std::vector<TrajectoryPoint> points)
      : trajectory_points_(std::move(points)) {}

  inline const DataType &data() const { return trajectory_points_; }

 protected:
  std::vector<TrajectoryPoint> trajectory_points_;
};

}  // namespace planning
