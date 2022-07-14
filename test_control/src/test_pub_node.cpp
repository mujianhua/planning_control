/**
 * @author mujianhua
 * @brief
 */

#include <ros/ros.h>
#include <test_control/chassis_data.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_pub_node");
    ros::NodeHandle nh;
    test_control::chassis_data msg;

    ros::Publisher test_publisher_ =
        nh.advertise<test_control::chassis_data>("/chassis_data", 1);
    ros::Rate loop_rate(1);

    double count = 0.0;
    while (ros::ok()) {
        msg.simulink_time = count;
        msg.Xo = count;
        msg.Yo = count;
        msg.Theta = count;
        count += 0.5;

        test_publisher_.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}