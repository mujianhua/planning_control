cmake_minimum_required(VERSION 3.0.2)
project(test_common)


find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    rospy
    std_msgs
)

add_message_files(
    FILES
    PathPoint.msg
    TrajectoryPoint.msg
    ChassisData.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(CATKIN_DEPENDS
    message_runtime
    roscpp
    rospy
    std_msgs
    INCLUDE_DIRS include
    LIBRARIES vehicle_state_provider
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(vehicle_state_provider
    src/vehicle_state_provider.cpp
)
target_link_libraries(vehicle_state_provider
    ${catkin_LIBRARIES}
)


