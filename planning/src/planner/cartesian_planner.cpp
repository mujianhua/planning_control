#include "planner/cartesian_planner.h"
#include <string>
#include <ros/ros.h>
#include "planner/dp_plan.h"
#include "planner/planner.h"
#include "ros/time.h"
#include "trajectory_optimization/trajectory_optimizer.h"
#include "visualization/plot.h"

namespace mujianhua {
namespace planning {

CartesianPlanner::CartesianPlanner(
    const std::shared_ptr<DependencyInjector> &injector)
    : Planner(injector) {}

bool CartesianPlanner::Init(const PlanningConfig &config) {
    dp_ = new DPPlan(config);
    config_ = config;
    return true;
}

bool CartesianPlanner::Plan(const State &start_state, Frame *frame,
                            DiscretizedTrajectory *ptr_computed_trajectory) {

    ros::Time start_time = ros::Time::now();
    ros::Time current_time = start_time;

    DiscretizedTrajectory coarse_trajectory;
    dp_->Plan(start_state, frame, coarse_trajectory);

    ROS_DEBUG("[CartesianPlanner] dp time is %f",
              (ros::Time::now() - current_time).toSec());
    current_time = ros::Time::now();

    std::vector<double> coarse_x, coarse_y;
    for (auto &pt : *coarse_trajectory.trajectory_points()) {
        coarse_x.push_back(pt.x);
        coarse_y.push_back(pt.y);
    }

    visualization::Plot(coarse_x, coarse_y, 0.1, visualization::Color::Cyan, 1,
                        "Coarse Trajectory");
    visualization::PlotPoints(coarse_x, coarse_y, 0.3,
                              visualization::Color::Cyan, 2,
                              "Coarse Trajectory");
    visualization::Trigger();

    opti_ = new TrajectoryOptimizer(config_, frame);
    OptiConstraints opti_constraints(
        start_state.x, start_state.y, start_state.theta, start_state.v,
        start_state.a, start_state.phi, start_state.omega);
    OptiStates optimized;
    opti_->OptimizeIteratively(coarse_trajectory, opti_constraints, optimized);

    return true;
}

} // namespace planning
} // namespace mujianhua