#include "planning_node.h"
#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "common/dependency_injector.h"
#include "common/discretized_trajectory.h"
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "common/obstacle.h"
#include "common/vehicle_state.h"
#include "config/planning_config.h"
#include "geometry_msgs/PoseStamped.h"
#include "on_lane_planning.h"
#include "planning/CenterLine.h"
#include "planning/DynamicObstacles.h"
#include "planning/Obstacles.h"
#include "planning_base.h"
#include "reference_line/reference_line.h"
#include "visualization/plot.h"

namespace mujianhua {
namespace planning {

PlanningNode::PlanningNode(const ros::NodeHandle &nh) : nh_(nh) {
    PlanningConfig planning_config;

    injector_ = std::make_shared<DependencyInjector>();
    planning_ = std::make_shared<OnLanePlanning>(injector_);
    planning_->Init(planning_config);
    obstacles_ = std::make_shared<IndexedObstacles>();

    reference_line_subscriber_ = nh_.subscribe(
        "/center_line", 1, &PlanningNode::CenterLineCallback, this);
    obstacles_subscriber_ =
        nh_.subscribe("/obstacles", 1, &PlanningNode::ObstaclesCallback, this);
    dynamic_obstacles_subscriber_ = nh_.subscribe(
        "/dynamic_obstacles", 1, &PlanningNode::DynamicObstaclesCallback, this);
    goal_subscriber_ = nh_.subscribe("/move_base_simple/goal", 1,
                                     &PlanningNode::PlanCallback, this);
}

void PlanningNode::Proc() {
    State state(0.0, 0.0, 0.0, 5.0);
    local_view_.vehicle_state = std::make_shared<State>(state);
    local_view_.obstacles = obstacles_;

    planning_->UpdateReferenceLine(reference_line_);

    DiscretizedTrajectory adc_trajectory_pb;
    planning_->RunOnce(local_view_, &adc_trajectory_pb);
}

void PlanningNode::ObstaclesCallback(const ::planning::ObstaclesConstPtr &msg) {
    size_t count = 0;
    for (auto &obstacle : msg->obstacles) {
        std::vector<math::Vec2d> points;
        for (auto &pt : obstacle.points) {
            points.emplace_back(pt.x, pt.y);
        }
        visualization_static_obstacles_.emplace_back(points);
        Obstacle obs("static" + std::to_string(++count),
                     math::Polygon2d{points}, true);
        obstacles_->Add("static" + std::to_string(count), obs);
    }
    Visualize();
}

void PlanningNode::DynamicObstaclesCallback(
    const ::planning::DynamicObstaclesConstPtr &msg) {
    size_t count = 0;
    for (auto &obstacle : msg->obstacles) {
        DynamicObstacle dynamic_obstacle;
        for (auto &tp : obstacle.trajectory) {
            math::Pose coord(tp.x, tp.y, tp.theta);
            std::vector<math::Vec2d> points;
            for (auto &pt : obstacle.polygon.points) {
                points.push_back(coord.transform({pt.x, pt.y, 0.0}));
            }

            dynamic_obstacle.emplace_back(tp.time, math::Polygon2d{points});
        }
        visualization_dynamic_obstacles_.emplace_back(dynamic_obstacle);
        Obstacle obs("dynamic" + std::to_string(++count), dynamic_obstacle,
                     false);

        obstacles_->Add("dynamic" + std::to_string(count), obs);
    }
    Visualize();
}

void PlanningNode::CenterLineCallback(
    const ::planning::CenterLineConstPtr &msg) {
    std::vector<TrajectoryPoint> points;
    for (const auto pt : msg->points) {
        TrajectoryPoint p;
        p.s = pt.s;
        p.x = pt.x;
        p.y = pt.y;
        p.theta = pt.theta;
        p.kappa = pt.kappa;
        p.left_bound = pt.left_bound;
        p.right_bound = pt.right_bound;
        points.push_back(p);
    }
    reference_line_ = new ReferenceLine(points);
    Visualize();
}

void PlanningNode::PlanCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    Proc();
}

void PlanningNode::Visualize() {
    std::vector<double> lb_x, lb_y, ub_x, ub_y;

    for (auto &point : reference_line_->reference_points()) {
        auto lb = reference_line_->GetCartesian(point.s, point.left_bound);
        auto ub = reference_line_->GetCartesian(point.s, -point.right_bound);

        lb_x.push_back(lb.x());
        lb_y.push_back(lb.y());
        ub_x.push_back(ub.x());
        ub_y.push_back(ub.y());
    }

    visualization::Plot(lb_x, lb_y, 0.1, visualization::Color::Grey, 1,
                        "Road Left");
    visualization::Plot(ub_x, ub_y, 0.1, visualization::Color::Grey, 1,
                        "Road Right");

    int idx = 0;
    for (auto &obstacle : visualization_static_obstacles_) {
        visualization::PlotPolygon(obstacle, 0.1, visualization::Color::Magenta,
                                   idx++, "Obstacles");
    }

    // plot first frame of dynamic obstacles
    idx = 1;
    for (auto &obstacle : visualization_dynamic_obstacles_) {
        auto color = visualization::Color::fromHSV(
            int((double)idx / visualization_dynamic_obstacles_.size() * 320),
            1.0, 1.0);
        color.set_alpha(0.5);
        visualization::PlotPolygon(obstacle[0].second, 0.1, color, idx,
                                   "Online Obstacle");
        idx++;
    }

    visualization::Trigger();
}

} // namespace planning
} // namespace mujianhua
