#include <iostream>
#include <queue>
#include <ros/ros.h>
#include <serial/serial.h>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_serial");

    ros::NodeHandle n;

    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(9600);
    sp.setTimeout(to);

    std::queue<u_int8_t> q[1024];

    try {
        sp.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    if (sp.isOpen()) {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    } else {
        return -1;
    }

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        bool wait_read = sp.waitReadable();
        if (wait_read) {
            size_t n = sp.available();
            if (n != 0) {
                n = sp.read(&q, n);
            }
        }
    }
}