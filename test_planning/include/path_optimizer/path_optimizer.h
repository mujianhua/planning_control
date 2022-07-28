/**
 * @file
 * @brief
 */

#pragma once

#include <memory>
#include <vector>
#include <grid_map_core/grid_map_core.hpp>
#include "common/frame.h"
#include "common/planning_dependency_injector.h"
#include "common/vehicle_state2.h"
#include "common_me/TrajectoryPoint.h"
#include "config/planning_flags.h"
#include "data_struct/data_struct.h"
#include "data_struct/reference_path.h"
#include "glog/logging.h"
#include "path_smoother/reference_path_smoother.h"
#include "path_smoother/tension_smoother.h"
#include "reference_line/qp_spline_reference_smoother.h"
#include "reference_line/reference_line.h"
#include "solver/base_solver.h"
#include "tools/map.h"
#include "tools/vehicle_state.h"

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
    bool ProcessReferencePath();

    void ProcessInitState();

    void SetReferencePathLength();

    bool OptimizePath(std::vector<PathPoint> *final_path);

    ReferencePath *reference_path_;
    const Map *grid_map_;

    std::shared_ptr<PlanningDependencyInjector> injector_;
    std::shared_ptr<ReferenceLineSmoother> reference_line_smoother_;
    ReferenceLine reference_line_;
    Frame *frame_;

    VehicleState2 *start_state_, *target_state_;
};

} // namespace planning
} // namespace mujianhua
