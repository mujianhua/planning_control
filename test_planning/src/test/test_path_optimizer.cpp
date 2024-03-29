/**
 * @file
 * @brief Test Path Optimizer.
 */

#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <glog/logging.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros_viz_tools/ros_viz_tools.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "../../../planning/src/common/data_struct.h"
#include "common/math/math_util.h"
#include "config/planning_flags.h"
#include "eigen3/Eigen/Dense"
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include "path_optimizer/path_optimizer.h"
#include "ros/node_handle.h"
#include "ros_viz_tools/color.h"
#include "tools/eigen2cv.h"
#include "visualization_msgs/Marker.h"

using namespace mujianhua::planning;

std::vector<PathPoint> reference_path;
PathPoint start_point, end_point;
bool start_point_receive = false, end_point_receive = false,
     reference_point_receive = false;

void referenceCb(const geometry_msgs::PointStampedConstPtr &p) {
    if (start_point_receive && end_point_receive) {
        reference_path.clear();
    }
    PathPoint point;
    point.x = p->point.x;
    point.y = p->point.y;
    LOG(INFO) << "receive a reference point "
              << "x: " << point.x << " y: " << point.y;
    reference_path.emplace_back(point);
    start_point_receive = end_point_receive = false;
    reference_point_receive = reference_path.size() >= 6;
}

void startPointCb(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
    start_point.x = start->pose.pose.position.x;
    start_point.y = start->pose.pose.position.y;
    start_point.theta = tf::getYaw(start->pose.pose.orientation);
    LOG(INFO) << "receive effective start point "
              << "x: " << start_point.x << " y: " << start_point.y
              << " heading: " << start_point.theta;
    if (reference_point_receive) {
        start_point_receive = true;
    }
}

void goalPointCb(const geometry_msgs::PoseStampedConstPtr &end) {
    end_point.x = end->pose.position.x;
    end_point.y = end->pose.position.y;
    end_point.theta = tf::getYaw(end->pose.orientation);
    LOG(INFO) << "receive effective end point "
              << "x: " << end_point.x << " y: " << end_point.y
              << " heading: " << end_point.theta;
    if (reference_point_receive) {
        end_point_receive = true;
    }
}

void test() {
    reference_path.clear();
    PathPoint point1;
    point1.x = 64.7571;
    point1.y = -37.1454;
    reference_path.emplace_back(point1);
    PathPoint point2;
    point2.x = 63.1541;
    point2.y = -40.0129;
    reference_path.emplace_back(point2);
    PathPoint point3;
    point3.x = 61.5568;
    point3.y = -42.4973;
    reference_path.emplace_back(point3);
    PathPoint point4;
    point4.x = 59.1095;
    point4.y = -44.7919;
    reference_path.emplace_back(point4);
    PathPoint point5;
    point5.x = 55.9125;
    point5.y = -46.6246;
    reference_path.emplace_back(point5);
    PathPoint point6;
    point6.x = 51.821;
    point6.y = -47.1128;
    reference_path.emplace_back(point6);
    PathPoint point7;
    point7.x = 49.1763;
    point7.y = -50.2195;
    reference_path.emplace_back(point7);

    start_point.x = 65.0331;
    start_point.y = -37.5212;
    start_point.theta = -2.07483;

    end_point.x = 49.4499;
    end_point.y = -50.5613;
    end_point.theta = -1.87097;

    start_point_receive = true;
    end_point_receive = true;
    reference_point_receive = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_path_optimizer");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);

    std::string base_dir = ros::package::getPath("test_planning");
    auto log_dir = base_dir + "/log";

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = log_dir;

    auto image_file = base_dir + "/gridmap.png";
    cv::Mat img_src = cv::imread(image_file, CV_8UC1);
    grid_map::GridMap grid_map(
        std::vector<std::string>{"obstacle", "distance"});
    double resolution = 0.2;
    grid_map::GridMapCvConverter::initializeFromImage(
        img_src, resolution, grid_map, grid_map::Position::Zero());
    unsigned char OCCUPY = 0;
    unsigned char FREE = 255;
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
        img_src, "obstacle", grid_map, OCCUPY, FREE, 0.5);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary =
        grid_map.get("obstacle").cast<unsigned char>();
    cv::distanceTransform(eigen2cv(binary), eigen2cv(grid_map.get("distance")),
                          CV_DIST_L2, CV_DIST_MASK_PRECISE);
    grid_map.get("distance") *= resolution;
    grid_map.setFrameId("/map");

    // Publisher
    ros::Publisher map_publisher =
        nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);
    nav_msgs::OccupancyGrid message;
    grid_map::GridMapRosConverter::toOccupancyGrid(grid_map, "obstacle", FREE,
                                                   OCCUPY, message);
    map_publisher.publish(message);

    // Subscriber
    ros::Subscriber reference_sub =
        nh.subscribe("/clicked_point", 1, referenceCb);
    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, startPointCb);
    ros::Subscriber goal_sub =
        nh.subscribe("/move_base_simple/goal", 1, goalPointCb);

    // Markers initialization
    ros_viz_tools::RosVizTools markers(nh, "markers");
    std::string marker_frame_id = "/map";

    // test();

    ros::Rate rate(30.0);
    while (ros::ok()) {
        markers.clear();
        int id = 0;

        // cancel double click
        if (reference_path.size() >= 2) {
            const auto &p1 = reference_path[reference_path.size() - 2];
            const auto &p2 = reference_path.back();
            if (math::Distance(p1, p2) <= 0.001) {
                reference_path.clear();
            }
        }

        // display reference path point selected by mouse.
        visualization_msgs::Marker reference_marker =
            ros_viz_tools::RosVizTools::newSphereList(0.5, "reference point",
                                                      id++, ros_viz_tools::RED,
                                                      marker_frame_id);
        for (auto &point : reference_path) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 1.0;
            reference_marker.points.push_back(p);
        }
        markers.append(reference_marker);

        geometry_msgs::Vector3 scale;
        scale.x = 2.0;
        scale.y = 0.3;
        scale.z = 0.3;
        if (start_point_receive) {
            // display start and end point selected by mouse.
            geometry_msgs::Pose start_pose;
            start_pose.position.x = start_point.x;
            start_pose.position.y = start_point.y;
            start_pose.position.z = 1.0;
            auto start_quaternion =
                tf::createQuaternionFromYaw(start_point.theta);
            start_pose.orientation.x = start_quaternion.x();
            start_pose.orientation.y = start_quaternion.y();
            start_pose.orientation.z = start_quaternion.z();
            start_pose.orientation.w = start_quaternion.w();
            visualization_msgs::Marker start_marker =
                ros_viz_tools::RosVizTools::newArrow(
                    scale, start_pose, "start point", id++, ros_viz_tools::CYAN,
                    marker_frame_id);
            markers.append(start_marker);
        }
        if (end_point_receive) {
            geometry_msgs::Pose end_pose;
            end_pose.position.x = end_point.x;
            end_pose.position.y = end_point.y;
            end_pose.position.z = 1.0;
            auto end_quaternion = tf::createQuaternionFromYaw(end_point.theta);
            end_pose.orientation.x = end_quaternion.x();
            end_pose.orientation.y = end_quaternion.y();
            end_pose.orientation.z = end_quaternion.z();
            end_pose.orientation.w = end_quaternion.w();
            visualization_msgs::Marker end_marker =
                ros_viz_tools::RosVizTools::newArrow(
                    scale, end_pose, "end point", id++, ros_viz_tools::CYAN,
                    marker_frame_id);
            markers.append(end_marker);
        }

        std::vector<TrajectoryPoint> result_path;
        std::vector<PathPoint> smoothed_reference_line;
        if (start_point_receive && end_point_receive &&
            reference_point_receive) {
            PathOptimizer path_optimizer(start_point, end_point, grid_map);

            if (path_optimizer.Solve(reference_path, &result_path)) {
                ROS_INFO("Path optimize success!");
            }
            smoothed_reference_line =
                path_optimizer.GetReferenceLine().reference_points();

            start_point_receive = end_point_receive = reference_point_receive =
                false;
        }
        std::vector<PathPoint> optimization_path;
        for (const auto &point : result_path) {
            PathPoint p(point.path_point.x, point.path_point.y,
                        point.path_point.theta, point.path_point.kappa,
                        point.path_point.s);
            optimization_path.emplace_back(p);
        }

        // visualize result path
        ros_viz_tools::ColorRGBA path_color;
        path_color.r = 0.063;
        path_color.g = 0.305;
        path_color.b = 0.545;
        visualization_msgs::Marker result_path_marker =
            markers.newLineStrip(FLAGS_car_width, "optimized path", id++,
                                 path_color, marker_frame_id);
        for (const auto &point : optimization_path) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 1.0;
            result_path_marker.points.push_back(p);
            const auto k = point.kappa;
            path_color.a = std::min(fabs(k) / 0.15, 1.0);
            path_color.a = std::max((float)0.1, path_color.a);
            result_path_marker.colors.emplace_back(path_color);
        }
        markers.append(result_path_marker);

        // visualize smoothed reference line
        visualization_msgs::Marker smoothed_reference_line_marker =
            markers.newLineStrip(0.07, "smoothed reference line", id++,
                                 ros_viz_tools::YELLOW, marker_frame_id);
        for (auto &point : smoothed_reference_line) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 1.0;
            smoothed_reference_line_marker.points.push_back(p);
        }
        markers.append(smoothed_reference_line_marker);

        // visualize vehicle geometry
        ros_viz_tools::ColorRGBA vehicle_color = ros_viz_tools::GRAY;
        vehicle_color.a = 0.5;
        visualization_msgs::Marker vehicle_geometry_marker =
            markers.newLineList(0.03, "vehicle", id++, vehicle_color,
                                marker_frame_id);
        static const double length{FLAGS_car_length};
        static const double width{FLAGS_car_width};
        static const double rear_d{FLAGS_rear_length};
        static const double front_d{FLAGS_front_length};
        for (size_t i = 0; i != optimization_path.size(); ++i) {
            double heading = optimization_path[i].theta;
            PathPoint p1, p2, p3, p4;
            p1.x = front_d;
            p1.y = width / 2;
            p2.x = front_d;
            p2.y = -width / 2;
            p3.x = rear_d;
            p3.y = -width / 2;
            p4.x = rear_d;
            p4.y = width / 2;
            auto tmp_relto = optimization_path[i];
            tmp_relto.theta = heading;
            p1 = math::Local2Global(tmp_relto, p1);
            p2 = math::Local2Global(tmp_relto, p2);
            p3 = math::Local2Global(tmp_relto, p3);
            p4 = math::Local2Global(tmp_relto, p4);
            geometry_msgs::Point pp1, pp2, pp3, pp4;
            pp1.x = p1.x;
            pp1.y = p1.y;
            pp1.z = 0.1;
            pp2.x = p2.x;
            pp2.y = p2.y;
            pp2.z = 0.1;
            pp3.x = p3.x;
            pp3.y = p3.y;
            pp3.z = 0.1;
            pp4.x = p4.x;
            pp4.y = p4.y;
            pp4.z = 0.1;
            vehicle_geometry_marker.points.push_back(pp1);
            vehicle_geometry_marker.points.push_back(pp2);
            vehicle_geometry_marker.points.push_back(pp2);
            vehicle_geometry_marker.points.push_back(pp3);
            vehicle_geometry_marker.points.push_back(pp3);
            vehicle_geometry_marker.points.push_back(pp4);
            vehicle_geometry_marker.points.push_back(pp4);
            vehicle_geometry_marker.points.push_back(pp1);
        }
        markers.append(vehicle_geometry_marker);

        markers.publish();

        // Wait for next cycle.
        ros::spinOnce();
        rate.sleep();
    }
    google::ShutdownGoogleLogging();

    return 0;
}
