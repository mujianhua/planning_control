#include "planner/em_planner.h"

#include "task/dp_poly_path/dp_poly_path_optimizer.h"
#include "task/task.h"

namespace planning {

EMPlanner::EMPlanner(const PlanningConfig &config) : Planner(config_) {}

bool EMPlanner::Plan(const TrajectoryPoint &planning_init_point, Frame *frame,
                     DiscretizedTrajectory &result) {
  return false;
}

}  // namespace planning
