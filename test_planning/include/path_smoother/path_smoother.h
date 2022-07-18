/**
 * @file
 * @brief
 */

#pragma once

#include <string>
#include "common_me/TrajectoryPoint.h"

namespace mujianhua {
namespace planning {

using common_me::TrajectoryPoint;

class PathSmoother {
  public:
    PathSmoother() = delete;

    PathSmoother(std::string &type,
                 const std::vector<TrajectoryPoint> &input_points,
                 const TrajectoryPoint &start_point);

    virtual ~PathSmoother() = default;

  protected:
    const TrajectoryPoint start_point_;

  private:
    const std::vector<TrajectoryPoint> input_points_;
};

} // namespace planning
} // namespace mujianhua