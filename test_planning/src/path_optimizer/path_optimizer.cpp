#include <memory>
#include <vector>
#include "common/frame.h"
#include "common/planning_dependency_injector.h"
#include "common/vehicle_state2.h"
#include "common_me/TrajectoryPoint.h"
#include "data_struct/data_struct.h"
#include "math/math_util.h"
#include "path_optimizer/qp_path_optimizer.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_point.h"
#include "solver/base_solver.h"
#include "tools/vehicle_state.h"

namespace mujianhua {
namespace planning {

PathOptimizer::PathOptimizer(const PathPoint &start_point,
                             const PathPoint &end_point,
                             const grid_map::GridMap &map)
    : reference_path_(new ReferencePath), grid_map_(new Map{map}) {
    VehicleState2 start_state(start_point.x, start_point.y, start_point.theta);

    start_state_ =
        new VehicleState2(start_point.x, start_point.y, start_point.theta);
    target_state_ =
        new VehicleState2(end_point.x, end_point.y, end_point.theta);

    reference_line_ = new ReferenceLine();

    reference_line_smoother_ =
        std::make_shared<QPSplineReferenceLineSmoother>();
}

PathOptimizer::~PathOptimizer() { delete reference_path_; }

bool PathOptimizer::Solve(const std::vector<PathPoint> &raw_reference_points,
                          std::vector<TrajectoryPoint> *final_path) {
    CHECK_NOTNULL(final_path);

    frame_ = new Frame(*start_state_, *target_state_, *grid_map_);

    ROS_DEBUG("reference line smoother is %s",
              reference_line_smoother_->Name().c_str());
    reference_line_->Clear();
    reference_line_smoother_->Smooth(raw_reference_points, reference_line_,
                                     frame_);

    if (!ProcessReferencePath()) {
        ROS_ERROR("process reference path FAILED.");
    }

    std::vector<PathPoint> optimization_path;

    OptimizePath(&optimization_path);

    final_path->clear();
    for (const auto &point : optimization_path) {
        TrajectoryPoint tmp_point;
        tmp_point.path_point.x = point.x;
        tmp_point.path_point.y = point.y;
        tmp_point.path_point.theta = point.theta;
        tmp_point.path_point.kappa = point.kappa;
        tmp_point.path_point.s = point.s;
        final_path->emplace_back(tmp_point);
    }

    return true;
}

bool PathOptimizer::ProcessReferencePath() {
    ProcessInitState();
    SetReferencePathLength();
    if (!reference_line_->BuildReferenceLineBySpline(FLAGS_output_spacing / 2.0,
                                                     FLAGS_output_spacing)) {
        ROS_ERROR("unable build reference from spline");
    }
    reference_line_->UpdateBounds(frame_);
    return true;
}

void PathOptimizer::ProcessInitState() {
    ReferencePoint init_point = reference_line_->GetRerencePoint(0.0);
    // TODO:
    auto first_point_local =
        math::Global2Local(init_point, *frame_->GetVehicleStartState());
    double min_distance =
        math::Distance(init_point, *frame_->GetVehicleStartState());
    double initial_offset =
        first_point_local.y() < 0.0 ? min_distance : -min_distance;
    double initial_heading_error = math::ConstrainAngle(
        frame_->GetVehicleStartState()->heading() - init_point.theta());
    VehicleState2 start_state = *frame_->GetVehicleStartState();
    // TODO:
    start_state.SetInitialError(initial_offset, initial_heading_error);
    frame_->UpdateVehicleStartState(start_state);
}

void PathOptimizer::SetReferencePathLength() {
    ReferencePoint end_ref_point =
        reference_line_->GetRerencePoint(reference_line_->GetLength());

    auto vehicle_target_to_end_ref_point =
        math::Global2Local(end_ref_point, *target_state_);
    if (vehicle_target_to_end_ref_point.x() > 0.0)
        return;
    auto target_projection = reference_line_->FindProjectPoint(
        target_state_->x(), target_state_->y());
    reference_line_->SetLength(target_projection.s());
}

bool PathOptimizer::OptimizePath(std::vector<PathPoint> *final_path) {
    CHECK_NOTNULL(final_path);
    final_path->clear();
    QPPathOptimizer pre_solver(reference_line_, frame_, false);
    if (!pre_solver.Solve(final_path)) {
        ROS_ERROR("pre solving failed!");
    }

    reference_line_->BuildReferenceFromStates(*final_path);
    reference_line_->UpdateBounds(frame_);

    VehicleState2 start_state = *frame_->GetVehicleStartState();
    start_state.SetInitialError(0.0, 0.0);
    frame_->UpdateVehicleStartState(start_state);
    VehicleState2 target_state = *frame_->GetVehicleTargetState();
    target_state.Update(final_path->back().x, final_path->back().y,
                        final_path->back().theta);
    frame_->UpdateVehicleTargetState(target_state);

    QPPathOptimizer solver(reference_line_, frame_, true);
    solver.Solve(final_path);

    return true;
}

const ReferenceLine &PathOptimizer::GetReferenceLine() const {
    return *reference_line_;
}

} // namespace planning
} // namespace mujianhua
