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
#include "planning/data_struct.h"

#include "math/vec2d.h"

namespace planning {

using math::Vec2d;

/**
 * Discretized Trajectory
 */
class ReferenceLine {
  public:
    ReferenceLine() = default;

    ReferenceLine(const ReferenceLine &rhs, size_t begin, size_t end = -1);

    explicit ReferenceLine(std::vector<TrajectoryPoint> points)
        : reference_points_(std::move(points)) {}

    inline const std::vector<TrajectoryPoint> &reference_points() const {
        return reference_points_;
    }

    TrajectoryPoint GetMatchPoint(double station) const;

    TrajectoryPoint GetProjection(const Vec2d &xy) const;

    Vec2d XYToSL(const Vec2d &xy) const;

    Vec2d GetCartesian(double station, double lateral) const;

  private:
    std::vector<TrajectoryPoint>::const_iterator
    QueryLowerBoundStationPoint(double station) const;

    std::vector<TrajectoryPoint>::const_iterator
    QueryNearestPoint(const Vec2d &point, double *out_distance = nullptr) const;

  private:
    std::vector<TrajectoryPoint> reference_points_;
};

} // namespace planning
