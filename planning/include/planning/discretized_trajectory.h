/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source
 *codes. Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#pragma once

#include <cassert>
#include <utility>
#include <vector>

#include "math/vec2d.h"
#include "planning/data_struct.h"

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

} // namespace planning
