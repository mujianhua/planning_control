#include "path_optimizer/path_optimizer.h"

namespace mujianhua {
namespace planning {

PathOptimizer::PathOptimizer(const TrajectoryPoint &start_point,
                             const TrajectoryPoint &end_point,
                             const grid_map::GridMap &map)
    : vehicle_state_(new VehicleState{start_point, end_point}),
      reference_path_(new ReferencePath) {}

PathOptimizer::~PathOptimizer() {
    delete vehicle_state_;
    delete reference_path_;
}

bool PathOptimizer::Solve(const std::vector<TrajectoryPoint> &initial_path,
                          std::vector<TrajectoryPoint> *final_path) {
    CHECK_NOTNULL(final_path);

    auto t1 = ros::Time::now();
    if (initial_path.empty()) {
        LOG(ERROR) << "initial path is empty, quit path optimization!";
        return false;
    }
    auto reference_path_smoother = ReferencePathSmoother::Creat(
        FLAGS_smoothing_method, initial_path, vehicle_state_->getStartPoint());

    return true;
}

} // namespace planning
} // namespace mujianhua