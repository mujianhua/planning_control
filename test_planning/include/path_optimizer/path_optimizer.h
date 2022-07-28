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

    PathOptimizer(const TrajectoryPoint &start_point,
                  const TrajectoryPoint &end_point,
                  const grid_map::GridMap &map);

    ~PathOptimizer();

    bool Solve(const std::vector<TrajectoryPoint> &reference_points,
               std::vector<TrajectoryPoint> *final_path);

    const ReferencePath &GetReferencePath() const;

  private:
    bool ProcessReferencePath();

    bool ProcessReferencePath2();

    void ProcessInitState();

    void ProcessInitState2();

    void SetReferencePathLength();

    void SetReferencePathLength2();

    bool OptimizePath(std::vector<TrajectoryPoint> *final_path);

    bool OptimizePath2(std::vector<PathPoint> *final_path);

    VehicleState *vehicle_state_;
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
