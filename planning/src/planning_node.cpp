#include "planning_node.h"

namespace planning {

PlanningNode::PlanningNode(const ros::NodeHandle &nh) : nh_(nh) {
    frame_ = std::make_shared<Frame>(config_);

    if (FLAGS_planner == "Cartesian") {
        planner_ = std::make_shared<CartesianPlanner>(config_);
    }

    center_line_subscriber_ = nh_.subscribe(
        "/center_line", 1, &PlanningNode::CenterLineCallback, this);
    obstacles_subscriber_ = nh_.subscribe(
        "/obstacles", 1, &PlanningNode::StaticObstaclesCallback, this);
    dynamic_obstacles_subscriber_ = nh_.subscribe(
        "/dynamic_obstacles", 1, &PlanningNode::DynamicObstaclesCallback, this);

    goal_subscriber_ = nh_.subscribe("/move_base_simple/goal", 1,
                                     &PlanningNode::PlanCallback, this);
}

void PlanningNode::CenterLineCallback(const CenterLineConstPtr &msg) {
    Trajectory data;
    for (auto &pt : msg->points) {
        TrajectoryPoint tp;
        tp.s = pt.s;
        tp.x = pt.x;
        tp.y = pt.y;
        tp.theta = pt.theta;
        tp.kappa = pt.kappa;
        tp.left_bound = pt.left_bound;
        tp.right_bound = pt.right_bound;
        data.push_back(tp);
    }

    frame_->SetReferenceLine(ReferenceLine(data));
    frame_->Visualize();
}

// TODO: 处理不同plan中的相同障碍物,进行编号
void PlanningNode::StaticObstaclesCallback(const ObstaclesConstPtr &msg) {
    ROS_DEBUG("[Planning Node] receive static obstacles message.");
    frame_->ClearStaticObstacles();
    size_t count = 0;
    for (auto &obstacle : msg->obstacles) {
        std::vector<math::Vec2d> points;
        for (auto &pt : obstacle.points) {
            points.emplace_back(pt.x, pt.y);
        }
        frame_->AddObstacle("static" + std::to_string(++count),
                            math::Polygon2d(points));
    }
    frame_->Visualize();
}

void PlanningNode::DynamicObstaclesCallback(
    const DynamicObstaclesConstPtr &msg) {
    ROS_DEBUG("[Planning Node] receive dynamic obstacles message.");
    frame_->ClearDynamicObstacles();
    size_t count = 0;
    for (auto &obstacle : msg->obstacles) {
        Frame::DynamicObstacle dynamic_obstacle;
        for (auto &tp : obstacle.trajectory) {
            math::Pose coord(tp.x, tp.y, tp.theta);
            std::vector<math::Vec2d> points;
            for (auto &pt : obstacle.polygon.points) {
                points.push_back(coord.transform({pt.x, pt.y, 0.0}));
            }
            math::Polygon2d polygon(points);

            dynamic_obstacle.emplace_back(tp.time, points);
        }
        // TODO:
        frame_->AddObstacle("dynamic" + std::to_string(++count),
                            dynamic_obstacle);
    }
    frame_->Visualize();
}

void PlanningNode::PlanCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    DiscretizedTrajectory result;

    state_ = CartesianPlanner::StartState(0.0, 0.0, 0.0, 5.0);

    if (planner_->Plan(state_, frame_, result)) {
        double dt = config_.tf / (double)(config_.nfe - 1);
        for (int i = 0; i < config_.nfe; i++) {
            double time = dt * i;
            auto dynamic_obstacles = frame_->QueryDynamicObstacles(time);
            for (auto &obstacle : dynamic_obstacles) {
                int hue = int((double)obstacle.first /
                              frame_->index_dynamic_obstacles().Size() * 320);

                visualization::PlotPolygon(
                    obstacle.second, 0.2,
                    visualization::Color::fromHSV(hue, 1.0, 1.0),
                    obstacle.first, "Online Obstacle");
            }

            auto &pt = result.data().at(i);
            PlotVehicle(1, {pt.x, pt.y, pt.theta},
                        atan(pt.kappa * config_.vehicle.wheel_base));
            ros::Duration(dt).sleep();
        }

        visualization::Trigger();
    }
}

void PlanningNode::PlotVehicle(int id, const math::Pose &pt, double phi) {
    auto tires = GenerateTireBoxes({pt.x(), pt.y(), pt.theta()}, phi);

    int tire_id = 1;
    for (auto &tire : tires) {
        visualization::PlotPolygon(math::Polygon2d(tire), 0.1,
                                   visualization::Color::White,
                                   id * (tire_id++), "Tires");
    }
    visualization::PlotPolygon(math::Polygon2d(config_.vehicle.GenerateBox(
                                   {pt.x(), pt.y(), pt.theta()})),
                               0.2, visualization::Color::Yellow, id,
                               "Footprint");
    visualization::Trigger();
}

std::array<math::Box2d, 4>
PlanningNode::GenerateTireBoxes(const math::Pose pose, double phi) const {
    auto front_pose = pose.extend(config_.vehicle.wheel_base);
    auto track_width = config_.vehicle.width - 0.195;
    double rear_track_width_2 = track_width / 2,
           front_track_width_2 = track_width / 2;
    double box_length = 0.6345;
    double sin_t = sin(pose.theta());
    double cos_t = cos(pose.theta());
    return {
        math::Box2d({pose.x() - rear_track_width_2 * sin_t,
                     pose.y() + rear_track_width_2 * cos_t},
                    pose.theta(), box_length, 0.195),
        math::Box2d({pose.x() + rear_track_width_2 * sin_t,
                     pose.y() - rear_track_width_2 * cos_t},
                    pose.theta(), box_length, 0.195),
        math::Box2d({front_pose.x() - front_track_width_2 * sin_t,
                     front_pose.y() + front_track_width_2 * cos_t},
                    front_pose.theta() + phi, box_length, 0.195),
        math::Box2d({front_pose.x() + front_track_width_2 * sin_t,
                     front_pose.y() - front_track_width_2 * cos_t},
                    front_pose.theta() + phi, box_length, 0.195),
    };
}

} // namespace planning

int main(int argc, char **argv) {

    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);

    planning::visualization::Init(nh, "map", "planning_markers");

    planning::PlanningNode node(nh);

    ros::spin();
    return 0;
}
