#include "planner/cartesian_planner.h"
#include <string>
#include <vector>
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

    std::vector<double> opti_x, opti_y, opti_v;
    std::vector<TrajectoryPoint> result_data;
    double incremental_s = 0.0;
    for (int i = 0; i < config_.nfe; i++) {
        TrajectoryPoint tp;
        incremental_s += i > 0 ? hypot(optimized.x[i] - optimized.x[i - 1],
                                       optimized.y[i] - optimized.y[i - 1])
                               : 0.0;
        tp.s = incremental_s;

        tp.x = optimized.x[i];
        tp.y = optimized.y[i];
        tp.theta = optimized.theta[i];
        tp.velocity = optimized.v[i];
        tp.kappa = tan(optimized.phi[i]) / config_.vehicle.wheel_base;

        opti_x.push_back(tp.x);
        opti_y.push_back(tp.y);
        opti_v.push_back(tp.velocity);

        result_data.push_back(tp);
    }

    visualization::PlotTrajectory(
        opti_x, opti_y, opti_v, config_.vehicle.max_velocity, 0.1,
        visualization::Color::Green, 1, "Optimized Trajectory");
    visualization::Trigger();

    return true;
}

} // namespace planning
} // namespace mujianhua
