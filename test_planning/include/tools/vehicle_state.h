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

    void SetStartPoint(const TrajectoryPoint &point);

    void SetTargetPoint(const TrajectoryPoint &point);

    const TrajectoryPoint &getStartPoint() const;

    const TrajectoryPoint &getTargetPoint() const;

    void SetInitError(double init_offset, double init_heading_error);

    std::vector<double> GetInitError() const;

  private:
    TrajectoryPoint start_point_, end_point_;
    double init_offset_{};
    double init_heading_error_{};
};

} // namespace planning
} // namespace mujianhua