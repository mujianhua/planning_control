/**
 * @file
 * @brief
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include "OsqpEigen/OsqpEigen.h"
#include "common_me/TrajectoryPoint.h"
#include "data_struct/reference_path.h"
#include "glog/logging.h"
#include "math/math_util.h"
#include "tinyspline_ros/tinysplinecpp.h"
#include "tools/map.h"

namespace mujianhua {
namespace planning {

using common_me::TrajectoryPoint;

class ReferencePathSmoother {
  public:
    ReferencePathSmoother() = delete;

    ReferencePathSmoother(const std::vector<TrajectoryPoint> &initial_points,
                          const TrajectoryPoint &start_point,
                          const Map &grid_map);

    virtual ~ReferencePathSmoother() = default;

    // TODO
    static std::unique_ptr<ReferencePathSmoother>
    Creat(std::string &type, const std::vector<TrajectoryPoint> &initial_points,
          const TrajectoryPoint &start_point, const Map &grid_map);

    bool Solve(ReferencePath *reference_path);

    void bSpline();

  protected:
    bool SegmentRawReference(std::vector<double> *x_list,
                             std::vector<double> *y_list,
                             std::vector<double> *s_list,
                             std::vector<double> *heading_list,
                             std::vector<double> *kappa_list) const;

    const Map &grid_map_;
    const TrajectoryPoint start_point_;
    std::vector<double> x_list_, y_list_, s_list_;

  private:
    virtual bool Smooth(ReferencePath *reference_path) = 0;

    bool GraphSearchDp(ReferencePath *referemce_path);

    const std::vector<TrajectoryPoint> initial_points_;

    // Sample points in searching process.
    std::vector<double> layers_s_list_;
    std::vector<std::pair<double, double>> layers_bounds_;
};

} // namespace planning
} // namespace mujianhua
