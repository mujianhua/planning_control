#include "on_lane_planning.h"
#include <memory>
#include <ros/ros.h>
#include "common/frame.h"
#include "planner/cartesian_planner.h"
#include "planning_base.h"
#include "reference_line/reference_line_provider.h"

namespace mujianhua {
namespace planning {

bool OnLanePlanning::Init(const PlanningConfig &config) {
    PlanningBase::Init(config);

    // TODO:
    reference_line_provider_ = std::make_unique<ReferenceLineProvider>();

    planner_ = std::make_unique<CartesianPlanner>(injector_);
    planner_->Init(config_);

    ROS_INFO("planner is %s", planner_->Name().c_str());
    return true;
}

bool OnLanePlanning::InitFrame(const uint32_t sequence_num) {
    frame_ = std::make_unique<Frame>(sequence_num, reference_line_, local_view_,
                                     config_);
    if (frame_ == nullptr) {
        ROS_ERROR("Fail to init frame: nullptr.");
        return false;
    }

    return true;
}

void OnLanePlanning::RunOnce(const LocalView &local_view,
                             DiscretizedTrajectory *const adc_trajectory) {
    ROS_DEBUG("NOW begin to plan!");

    local_view_ = local_view;

    const auto frame_num = static_cast<uint32_t>(seq_num_++);
    if (!InitFrame(frame_num)) {
        ROS_ERROR("Fail to init frame");
    }
    ROS_DEBUG("[OnLanePlanning] init frame done.");

    planner_->Plan(*local_view.vehicle_state, frame_.get(), adc_trajectory);
}

} // namespace planning
} // namespace mujianhua
