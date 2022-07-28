#include <iostream>
#include <queue>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <serial/serial.h>
#include "serial_demo/wit_serial.h"

using namespace mujianhua::serial;

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_serial");

    ros::NodeHandle n;

    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(9600);
    sp.setTimeout(to);

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

    WitSerial wit_serial;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        bool wait_read = sp.waitReadable();
        if (wait_read) {
            std::vector<uint8_t> buffer;
            size_t n = sp.available();
            if (n != 0) {
                n = sp.read(buffer, n);
            }
            ROS_INFO("buffer size: %lu", buffer.size());
            wit_serial.Update(&buffer);
            wit_serial.ProcessData();
        }
        loop_rate.sleep();
    }
    sp.close();
    return 0;
}