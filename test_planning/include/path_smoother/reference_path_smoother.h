/**
 * @file
 * @brief
 */

#pragma once

#include <memory>
#include <string>
#include "common_me/TrajectoryPoint.h"
#include "data_struct/reference_path.h"
#include "glog/logging.h"
#include "math/math_util.h"
#include "tinyspline_ros/tinysplinecpp.h"

namespace mujianhua {
namespace planning {

using common_me::TrajectoryPoint;

class ReferencePathSmoother {
  public:
    ReferencePathSmoother() = delete;

    ReferencePathSmoother(const std::vector<TrajectoryPoint> &initial_points,
                          const TrajectoryPoint &start_point);

    virtual ~ReferencePathSmoother() = default;

    // TODO
    static std::unique_ptr<ReferencePathSmoother>
    Creat(std::string &type, const std::vector<TrajectoryPoint> &initial_points,
          const TrajectoryPoint &start_point);

    bool Solve(ReferencePath *reference_path);

    void bSpline();

  protected:
    const TrajectoryPoint start_point_;
    std::vector<double> x_list_, y_list_, s_list_;

  private:
    virtual bool Smooth(ReferencePath *reference_path) = 0;

    const std::vector<TrajectoryPoint> initial_points_;
};

} // namespace planning
} // namespace mujianhua