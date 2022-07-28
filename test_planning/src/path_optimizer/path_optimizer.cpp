#include <memory>
#include <vector>
#include "common/frame.h"
#include "common/planning_dependency_injector.h"
#include "common/vehicle_state2.h"
#include "common_me/TrajectoryPoint.h"
#include "math/math_util.h"
#include "path_optimizer/qp_path_optimizer.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_point.h"
#include "solver/base_solver.h"
#include "tools/vehicle_state.h"

namespace mujianhua {
namespace planning {

PathOptimizer::PathOptimizer(const TrajectoryPoint &start_point,
                             const TrajectoryPoint &end_point,
                             const grid_map::GridMap &map)
    : vehicle_state_(new VehicleState{start_point, end_point}),
      reference_path_(new ReferencePath), grid_map_(new Map{map}) {
    VehicleState2 start_state(start_point.path_point.x,
                              start_point.path_point.y,
                              start_point.path_point.theta);

    start_state_ =
        new VehicleState2(start_point.path_point.x, start_point.path_point.y,
                          start_point.path_point.theta);
    target_state_ =
        new VehicleState2(end_point.path_point.x, end_point.path_point.y,
                          end_point.path_point.theta);

    reference_line_smoother_ =
        std::make_shared<QPSplineReferenceLineSmoother>();
}

PathOptimizer::~PathOptimizer() {
    delete vehicle_state_;
    delete reference_path_;
}

bool PathOptimizer::Solve(const std::vector<TrajectoryPoint> &reference_points,
                          std::vector<TrajectoryPoint> *final_path) {
    CHECK_NOTNULL(final_path);

    // test
    VehicleState2 vehicle_state(
        vehicle_state_->getStartPoint().path_point.x,
        vehicle_state_->getStartPoint().path_point.y,
        vehicle_state_->getStartPoint().path_point.theta);
    frame_ = new Frame(vehicle_state, *target_state_, *grid_map_);
    std::vector<ReferencePoint> raw_reference_points;
    for (const auto &reference_point : reference_points) {
        ReferencePoint ref_point(reference_point.path_point.x,
                                 reference_point.path_point.y);
        raw_reference_points.emplace_back(ref_point);
    }

    ROS_DEBUG("reference line smoother is %s",
              reference_line_smoother_->Name().c_str());
    reference_line_.Clear();
    reference_line_smoother_->Smooth(raw_reference_points, reference_line_,
                                     frame_);
    // test end

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

    if (!ProcessReferencePath2()) {
        ROS_ERROR("process reference path FAILED.");
    }

    std::vector<PathPoint> optimization_path;

    OptimizePath2(&optimization_path);

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
    if (!reference_path_->BuildReferenceFromSpline(FLAGS_output_spacing / 2.0,
                                                   FLAGS_output_spacing)) {
        ROS_ERROR("unable build reference from spline");
    }
    reference_path_->UpdateBounds(*grid_map_);
    return true;
}

bool PathOptimizer::ProcessReferencePath2() {
    ProcessInitState2();
    SetReferencePathLength2();
    if (!reference_line_.BuildReferenceLineBySpline(FLAGS_output_spacing / 2.0,
                                                    FLAGS_output_spacing)) {
        ROS_ERROR("unable build reference from spline");
    }
    reference_line_.UpdateBounds(frame_);
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

void PathOptimizer::ProcessInitState2() {
    ReferencePoint init_point = reference_line_.GetRerencePoint(0.0);
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

void PathOptimizer::SetReferencePathLength2() {
    ReferencePoint end_ref_point =
        reference_line_.GetRerencePoint(reference_line_.GetLength());

    auto vehicle_target_to_end_ref_point =
        math::Global2Local(end_ref_point, *target_state_);
    if (vehicle_target_to_end_ref_point.x() > 0.0)
        return;
    auto target_projection = reference_line_.FindProjectPoint(
        target_state_->x(), target_state_->y());
    reference_line_.SetLength(target_projection.s());
}

bool PathOptimizer::OptimizePath(std::vector<TrajectoryPoint> *final_path) {
    CHECK_NOTNULL(final_path);
    final_path->clear();
    BaseSolver pre_solver(reference_path_, vehicle_state_, 0, false);
    if (!pre_solver.Solve(final_path)) {
        ROS_ERROR("pre solving failed!");
    }

    reference_path_->BuildReferenceFromStates(*final_path);
    reference_path_->UpdateBounds(*grid_map_);
    vehicle_state_->SetStartPoint(final_path->front());
    vehicle_state_->SetTargetPoint(final_path->back());
    vehicle_state_->SetInitError(0.0, 0.0);

    BaseSolver solver(reference_path_, vehicle_state_, 0, true);
    solver.Solve(final_path);

    return true;
}

bool PathOptimizer::OptimizePath2(std::vector<PathPoint> *final_path) {
    CHECK_NOTNULL(final_path);
    final_path->clear();
    QPPathOptimizer pre_solver(&reference_line_, frame_, false);
    if (!pre_solver.Solve(final_path)) {
        ROS_ERROR("pre solving failed!");
    }

    reference_line_.BuildReferenceFromStates(*final_path);
    reference_line_.UpdateBounds(frame_);

    VehicleState2 start_state = *frame_->GetVehicleStartState();
    start_state.SetInitialError(0.0, 0.0);
    frame_->UpdateVehicleStartState(start_state);
    VehicleState2 target_state = *frame_->GetVehicleTargetState();
    target_state.Update(final_path->back().x, final_path->back().y,
                        final_path->back().theta);
    frame_->UpdateVehicleTargetState(target_state);

    QPPathOptimizer solver(&reference_line_, frame_, true);
    solver.Solve(final_path);

    return true;
}

const ReferencePath &PathOptimizer::GetReferencePath() const {
    return *reference_path_;
}

} // namespace planning
} // namespace mujianhua
