/**
 * @file
 * @brief
 */

#pragma once

#include <memory>
#include <vector>
#include <grid_map_core/grid_map_core.hpp>
#include "common/data_struct.h"
#include "common/frame.h"
#include "common/planning_dependency_injector.h"
#include "common/vehicle_state.h"
#include "common_me/TrajectoryPoint.h"
#include "config/planning_flags.h"
#include "glog/logging.h"
#include "reference_line/qp_spline_reference_smoother.h"
#include "reference_line/reference_line.h"
#include "tools/map.h"

namespace mujianhua {
namespace planning {

using common_me::TrajectoryPoint;

class PathOptimizer {
  public:
    PathOptimizer() = delete;

    PathOptimizer(const PathPoint &start_point, const PathPoint &end_point,
                  const grid_map::GridMap &map);

    ~PathOptimizer();

    bool Solve(const std::vector<PathPoint> &reference_points,
               std::vector<TrajectoryPoint> *final_path);

    const ReferenceLine &GetReferenceLine() const;

  private:
    bool ProcessReferenceLine();

    void ProcessInitState();

    void SetReferenceLineLength();

    bool OptimizePath(std::vector<PathPoint> *final_path);

    const Map *grid_map_;

    std::shared_ptr<PlanningDependencyInjector> injector_;
    std::shared_ptr<ReferenceLineSmoother> reference_line_smoother_;
    ReferenceLine *reference_line_;
    Frame *frame_;

    VehicleState *start_state_, *target_state_;
};

} // namespace planning
} // namespace mujianhua
