#include "on_lane_planning.h"
#include <memory>
#include "planner/cartesian_planner.h"
#include "planning/frame.h"
#include "planning/planning_config.h"
#include "planning/reference_line.h"
#include "planning_base.h"
#include "visualization/plot.h"

namespace planning {

OnLanePlanning::OnLanePlanning(const PlanningConfig &config)
    : PlanningBase(config) {
  if (FLAGS_planner == "Cartesian") {
    planner_ = std::make_unique<CartesianPlanner>(config_);
  }
  frame_ = std::make_unique<Frame>(config_);
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>();

  reference_line_provider_->Start();
}

OnLanePlanning::~OnLanePlanning() { reference_line_provider_->Stop(); }

void OnLanePlanning::UpdateFrame() {
  // obstacles
  frame_->ClearObstacles();
  frame_->index_dynamic_obstacles() = *local_view_.dynamic_obstacle;
  frame_->index_static_obstacles() = *local_view_.static_obstacle;

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

  if (!planner_->Plan(*local_view.vehicle_state, frame_.get(),
                      *adc_trajectory)) {
    ROS_ERROR("unable plan a trajectory!");
  }

  reference_line_provider_->Stop();
}

}  // namespace planning
