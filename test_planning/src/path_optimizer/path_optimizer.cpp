#include "path_optimizer/path_optimizer.h"
#include "math/math_util.h"
#include "tools/vehicle_state.h"

namespace mujianhua {
namespace planning {

PathOptimizer::PathOptimizer(const TrajectoryPoint &start_point,
                             const TrajectoryPoint &end_point,
                             const grid_map::GridMap &map)
    : vehicle_state_(new VehicleState{start_point, end_point}),
      reference_path_(new ReferencePath), grid_map_(new Map{map}) {}

PathOptimizer::~PathOptimizer() {
    delete vehicle_state_;
    delete reference_path_;
}

bool PathOptimizer::Solve(const std::vector<TrajectoryPoint> &reference_points,
                          std::vector<TrajectoryPoint> *final_path) {
    CHECK_NOTNULL(final_path);

    auto t1 = ros::Time::now();
    if (reference_points.empty()) {
        LOG(ERROR) << "initial path is empty, quit path optimization!";
        return false;
    }
    reference_path_->clear();
    auto reference_path_smoother = ReferencePathSmoother::Creat(
        FLAGS_smoothing_method, reference_points,
        vehicle_state_->getStartPoint(), *grid_map_);
    if (!reference_path_smoother->Solve(reference_path_)) {
        ROS_ERROR("reference path optimization FAILED.");
    }

    if (!ProcessReferencePath()) {
        ROS_ERROR("process reference path FAILED.");
    }

    OptimizePath(final_path);

    return true;
}

bool PathOptimizer::ProcessReferencePath() {
    ProcessInitState();
    SetReferencePathLength();
    if (!reference_path_->BuildReferenceFromSpline(FLAGS_output_spacing / 2.0,
                                                   FLAGS_output_spacing)) {
        ROS_ERROR("unable build reference from spline");
    }
    reference_path_->UpdateBounds(*grid_map_);
    return true;
}

void PathOptimizer::ProcessInitState() {
    TrajectoryPoint init_point;
    init_point.path_point.x = reference_path_->GetXS(0.0);
    init_point.path_point.y = reference_path_->GetYS(0.0);
    init_point.path_point.theta = math::GetHeading(
        reference_path_->GetXS(), reference_path_->GetYS(), 0.0);
    // TODO:
    auto first_point_local =
        math::Global2Local(vehicle_state_->getStartPoint(), init_point);
    double min_distance =
        math::Distance(vehicle_state_->getStartPoint(), init_point);
    double initial_offset =
        first_point_local.path_point.y < 0.0 ? min_distance : -min_distance;
    double initial_heading_error =
        math::ConstrainAngle(vehicle_state_->getStartPoint().path_point.theta -
                             init_point.path_point.theta);
    vehicle_state_->SetInitError(initial_offset, initial_heading_error);
}

void PathOptimizer::SetReferencePathLength() {
    TrajectoryPoint end_ref_point;
    end_ref_point.path_point.x =
        reference_path_->GetXS(reference_path_->GetLength());
    end_ref_point.path_point.y =
        reference_path_->GetYS(reference_path_->GetLength());
    end_ref_point.path_point.theta =
        math::GetHeading(reference_path_->GetXS(), reference_path_->GetYS(),
                         reference_path_->GetLength());
    auto vehicle_target_to_end_ref_point =
        math::Global2Local(end_ref_point, vehicle_state_->getTargetPoint());
    if (vehicle_target_to_end_ref_point.path_point.x > 0.0)
        return;
    auto target_projection = math::GetProjectPoint(
        reference_path_->GetXS(), reference_path_->GetYS(),
        vehicle_state_->getTargetPoint().path_point.x,
        vehicle_state_->getTargetPoint().path_point.y,
        reference_path_->GetLength());
    reference_path_->SetLength(target_projection.path_point.s);
}

bool PathOptimizer::OptimizePath(std::vector<TrajectoryPoint> *final_path) {
    CHECK_NOTNULL(final_path);
    final_path->clear();
    BaseSolver pre_solver(reference_path_, vehicle_state_, 0, false);
}

} // namespace planning
} // namespace mujianhua
