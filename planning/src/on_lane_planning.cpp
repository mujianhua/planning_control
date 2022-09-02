/**
 * @file on_lane_planning.cpp
 */

#include "on_lane_planning.h"

#include <memory>

#include "common/planning_gflags.h"
#include "planner/cartesian_planner.h"
#include "planner/lattice_planner.h"
#include "planning/frame.h"
#include "planning_base.h"
#include "reference_line/reference_line.h"
#include "visualization/plot.h"

namespace planning {

OnLanePlanning::OnLanePlanning(const PlanningConfig &config)
    : PlanningBase(config) {
  if (FLAGS_planner == "Cartesian") {
    planner_ = std::make_unique<CartesianPlanner>(config_);
  } else if (FLAGS_planner == "Lattice") {
    planner_ = std::make_unique<LatticePlanner>(config_);
  }

  frame_ = std::make_unique<Frame>(config_);
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>();

  reference_line_provider_->Start();
}

OnLanePlanning::~OnLanePlanning() { reference_line_provider_->Stop(); }

void OnLanePlanning::UpdateFrame() {
  frame_->Update(local_view_);

  // reference line
  ReferenceLine reference_line;
  reference_line_provider_->GetReferenceLine(&reference_line);
  frame_->SetReferenceLine(reference_line);

  frame_->Visualize();
}

void OnLanePlanning::RunOnce(const LocalView &local_view,
                             DiscretizedTrajectory *const adc_trajectory) {
  local_view_ = local_view;

  UpdateFrame();

  TrajectoryPoint init_point;
  init_point.x = local_view.vehicle_state->x;
  init_point.y = local_view.vehicle_state->y;
  init_point.theta = local_view.vehicle_state->theta;
  if (!planner_->Plan(init_point, frame_.get(), *adc_trajectory)) {
    ROS_ERROR("unable plan a trajectory!");
  }

  reference_line_provider_->Stop();
}

}  // namespace planning
