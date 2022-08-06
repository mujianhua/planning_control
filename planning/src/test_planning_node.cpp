#include <memory>
#include <vector>
#include <ros/ros.h>
#include "common/dependency_injector.h"
#include "common/math/polygon2d.h"
#include "common/obstacle.h"
#include "geometry_msgs/PoseStamped.h"
#include "on_lane_planning.h"
#include "planning/CenterLine.h"
#include "planning/Obstacles.h"
#include "planning_base.h"
#include "reference_line/reference_line.h"

using namespace mujianhua::planning;

class TestPlanningNode {
  public:
    explicit TestPlanningNode(const ros::NodeHandle &nh) : nh_(nh) {
        injector_ = std::make_shared<common::DependencyInjector>();
        planning_ = std::make_shared<OnLanePlanning>(injector_);
        reference_line_subscriber_ = nh_.subscribe(
            "/center_line", 1, &TestPlanningNode::CenterLineCallback, this);
        obstacles_subscriber_ = nh_.subscribe(
            "/obstacles", 1, &TestPlanningNode::ObstaclesCallback, this);
        goal_subscriber_ = nh_.subscribe("/move_base_simple/goal", 1,
                                         &TestPlanningNode::PlanCallback, this);
    }

    void RunPlanning() {
        State state{};
        state.x = 0.0;
        state.y = 0.0;
        state.theta = 0.0;
        state.v = 5.0;
        state.phi = 0.0;
        state.a = 0.0;
        state.omega = 0.0;
        local_view_.vehicle_state = std::make_shared<State>(state);

        planning_->Init();
        planning_->UpdateReferenceLine(reference_line_);
        planning_->RunOnce(local_view_);
    }

    void ObstaclesCallback(const planning::ObstaclesConstPtr &msg) {

        for (auto &obstacle : msg->obstacles) {
            std::vector<common::math::Vec2d> points;
            for (auto &pt : obstacle.points) {
                points.emplace_back(pt.x, pt.y);
            }
            common::math::Polygon2d polygon(points);
            common::Obstacle obs("1", polygon, true);
        }
    }

    void CenterLineCallback(const planning::CenterLineConstPtr &msg) {
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
    }

    void PlanCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
        RunPlanning();
    }

  private:
    ros::NodeHandle nh_;
    std::shared_ptr<OnLanePlanning> planning_;
    std::shared_ptr<common::DependencyInjector> injector_;
    ros::Subscriber reference_line_subscriber_, obstacles_subscriber_,
        goal_subscriber_;
    ReferenceLine *reference_line_{};
    LocalView local_view_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_planning_node");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);
    TestPlanningNode node(nh);

    ros::spin();
}