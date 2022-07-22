/**
 * @file
 * @brief
 */

#pragma once

#include <vector>
#include "common_me/TrajectoryPoint.h"

namespace mujianhua {
namespace planning {

using common_me::TrajectoryPoint;

class VehicleState {
  public:
    VehicleState() = default;

    VehicleState(const TrajectoryPoint &start_point,
                 const TrajectoryPoint &end_point);

    const TrajectoryPoint &getStartPoint() const;

    const TrajectoryPoint &getTargetPoint() const;

    void SetInitError(double init_offest, double init_heading_error);

  private:
    TrajectoryPoint start_point_, end_point_;
    double init_offest_;
    double init_heading_error_;
};

} // namespace planning
} // namespace mujianhua