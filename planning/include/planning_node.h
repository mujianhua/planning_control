#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "common/dependency_injector.h"
#include "common/indexed_list.h"
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "common/obstacle.h"
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

using namespace mujianhua::planning;

class PlanningNode {
  public:
    explicit PlanningNode(const ros::NodeHandle &nh);

    void Proc();

    void ObstaclesCallback(const ::planning::ObstaclesConstPtr &msg);

    void
    DynamicObstaclesCallback(const ::planning::DynamicObstaclesConstPtr &msg);

    void CenterLineCallback(const ::planning::CenterLineConstPtr &msg);

    void PlanCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void Visualize();

  private:
    ros::NodeHandle nh_;
    ros::Subscriber reference_line_subscriber_, obstacles_subscriber_,
        dynamic_obstacles_subscriber_, goal_subscriber_;

    std::shared_ptr<OnLanePlanning> planning_;
    std::shared_ptr<DependencyInjector> injector_;
    ReferenceLine *reference_line_{};

    LocalView local_view_;
    std::shared_ptr<IndexedObstacles> obstacles_;

    std::vector<DynamicObstacle> visualization_dynamic_obstacles_;
    std::vector<math::Polygon2d> visualization_static_obstacles_;
};

} // namespace planning
} // namespace mujianhua
