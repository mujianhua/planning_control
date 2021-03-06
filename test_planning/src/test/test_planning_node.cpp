/**
 * @file
 * @brief
 */

#include <memory>
#include <ros/ros.h>
#include "gtest/gtest.h"
#include "common_me/PathPoint.h"
#include "common_me/TrajectoryPoint.h"
#include "common_me/status.h"
#include "common_me/vehicle_state_provider.h"
#include "test_planning/dependency_injector.h"
#include "test_planning/lattice_planner.h"

using namespace mujianhua::planning;
using mujianhua::common::Status;

class PlanningNode {
  public:
    explicit PlanningNode(const ros::NodeHandle &nh) : nh_(nh) {
        test();

        injector_ = std::make_shared<DependencyInjector>();

        planner_ = std::make_shared<LatticePlanner>(injector_);

        PlanningConfig planning_config{};

        if (!planner_->Init(planning_config).ok()) {
            ROS_ERROR("planner init error");
        }

        ROS_INFO("%s", planner_->Name().c_str());

        planning_pub_ =
            nh_.advertise<common_me::TrajectoryPoint>("planning", 1);
        chassis_sub_ = nh_.subscribe("/chassis_data", 1,
                                     &PlanningNode::ReceiveChassisDataCb, this);
    }

    void ReceiveChassisDataCb(const common_me::ChassisData &msg) {
        injector_->vehicle_state()->Update(&msg);
    }

    void test() {
        mujianhua::common::VehicleStateProvider vehicle_state;
        common_me::ChassisData *data;
        data->Vx = 36.0;
        vehicle_state.Update(data);
        ROS_INFO("%f", vehicle_state.vx());
    }

  private:
    ros::NodeHandle nh_;

    // publisher
    ros::Publisher planning_pub_;
    // subscriber
    ros::Subscriber chassis_sub_;

    std::shared_ptr<DependencyInjector> injector_;
    std::shared_ptr<Planner> planner_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_planning");
    ros::NodeHandle nh;

    PlanningNode node(nh);

    ros::spin();

    return 0;
}