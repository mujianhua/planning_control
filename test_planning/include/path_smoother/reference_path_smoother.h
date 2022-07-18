/**
 * @file
 * @brief
 */

#pragma once

#include <memory>
#include <string>
#include "common_me/TrajectoryPoint.h"

namespace mujianhua {
namespace planning {

using common_me::TrajectoryPoint;

class ReferencePathSmoother {
  public:
    ReferencePathSmoother() = delete;

    ReferencePathSmoother(const std::vector<TrajectoryPoint> &initial_path,
                 const TrajectoryPoint &start_point);

    virtual ~ReferencePathSmoother() = default;

    // TODO
    static std::unique_ptr<ReferencePathSmoother>
    Creat(std::string &type, const std::vector<TrajectoryPoint> &initial_path,
          const TrajectoryPoint &start_point);

  protected:
    const TrajectoryPoint start_point_;

  private:
    const std::vector<TrajectoryPoint> initial_path_;
};

} // namespace planning
} // namespace mujianhua