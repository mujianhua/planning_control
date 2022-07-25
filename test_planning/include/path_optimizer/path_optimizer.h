/**
 * @file
 * @brief
 */

#pragma once

#include <memory>
#include <vector>
#include <grid_map_core/grid_map_core.hpp>
#include "common_me/TrajectoryPoint.h"
#include "config/planning_flags.h"
#include "data_struct/reference_path.h"
#include "glog/logging.h"
#include "path_smoother/reference_path_smoother.h"
#include "path_smoother/tension_smoother.h"
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

    void ProcessInitState();

    void SetReferencePathLength();

    bool OptimizePath(std::vector<TrajectoryPoint> *final_path);

    VehicleState *vehicle_state_;
    ReferencePath *reference_path_;
    const Map *grid_map_;
};

} // namespace planning
} // namespace mujianhua
