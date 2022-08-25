/**
 * @file plot.h
 * @brief
 */
#pragma once

#include <mutex>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "color.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
#include "planning/vehicle_param.h"

namespace planning {
namespace visualization {

using math::Polygon2d;
using math::Vec2d;

using Vector = std::vector<double>;

void Init(ros::NodeHandle &node, const std::string &frame,
          const std::string &topic);

void Plot(const Vector &xs, const Vector &ys, double width = 0.1,
          Color color = Color(1, 1, 1), int id = -1,
          const std::string &ns = "");

void Plot(const Vector &xs, const Vector &ys, double width = 0.1,
          const std::vector<Color> &color = {}, int id = -1,
          const std::string &ns = "");

void PlotPolygon(const Vector &xs, const Vector &ys, double width = 0.1,
                 Color color = Color::White, int id = -1,
                 const std::string &ns = "");

void PlotPolygon(const Polygon2d &polygon, double width = 0.1,
                 Color color = Color::White, int id = -1,
                 const std::string &ns = "");

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs,
                    double max_velocity = 10.0, double width = 0.1,
                    const Color &color = Color::Blue, int id = -1,
                    const std::string &ns = "");

void PlotPoints(const Vector &xs, const Vector &ys, double width = 0.1,
                const Color &color = Color::White, int id = -1,
                const std::string &ns = "");

void Trigger();

void Clear();

}  // namespace visualization
}  // namespace planning
