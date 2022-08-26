/**
 * @file planning_node.h
 * @brief
 */

#include <ros/ros.h>

#include "common/data_struct.h"
#include "geometry_msgs/PoseStamped.h"
#include "planner/cartesian_planner.h"
#include "planner/planner.h"
#include "planning/CenterLine.h"
#include "planning/DynamicObstacles.h"
#include "planning/Obstacles.h"
#include "planning/discretized_trajectory.h"
#include "planning/indexed_list.h"
#include "planning/local_view.h"
#include "planning/obstacle.h"
#include "planning/planning_config.h"
#include "planning_base.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_line_provider.h"
#include "visualization/plot.h"

namespace planning {

class PlanningNode {
 public:
  explicit PlanningNode(const ros::NodeHandle &nh);

  ~PlanningNode() = default;

  // TODO: 处理不同plan中的相同障碍物,进行编号
  void StaticObstaclesCallback(const ObstaclesConstPtr &msg);

  void DynamicObstaclesCallback(const DynamicObstaclesConstPtr &msg);

  void Proc(const geometry_msgs::PoseStampedConstPtr &msg);

 private:
  void Animation(const DiscretizedTrajectory &plan_trajectory);

  void PlotVehicle(int id, const math::Pose &pt, double phi);

  std::array<math::Box2d, 4> GenerateTireBoxes(const math::Pose pose,
                                               double phi = 0.0) const;

 private:
  ros::NodeHandle nh_;
  planning::PlanningConfig config_;

  VehicleState vehicle_state_{};

  LocalView local_view_;

  std::unique_ptr<PlanningBase> planning_base_;

  IndexedList<std::string, Obstacle> index_obstacles_;

  ros::Subscriber obstacles_subscriber_, dynamic_obstacles_subscriber_,
      goal_subscriber_;
};

}  // namespace planning
