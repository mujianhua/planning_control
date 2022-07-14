// serial_demo.cpp
#include <iostream>
#include <queue>
#include <ros/ros.h>
#include <serial/serial.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;

    //创建一个serial对象
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(9600);
    //串口设置timeout
    sp.setTimeout(to);

    try {
        //打开串口
        sp.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    //判断串口是否打开成功
    if (sp.isOpen()) {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    } else {
        return -1;
    }

    while (sizeof(sp.read()) >= 0)
        ;

    std::queue<int> q[1024];
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if (n != 0) {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);

            for (int i = 0; i < n; i++) {
                // 16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
            //把数据发送回去
            sp.write(buffer, n);
        }
        loop_rate.sleep();
    }

    //关闭串口
    sp.close();

    return 0;
}