cmake_minimum_required(VERSION 3.0.2)
project(common_me)


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
    LIBRARIES status
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# vehicle_state_provider library
add_library(vehicle_state_provider
    src/vehicle_state_provider.cpp
)
target_link_libraries(vehicle_state_provider
    ${catkin_LIBRARIES}
)

# status library
add_library(status
    src/status.cpp)
target_link_libraries(status
    ${catkin_LIBRARIES}
)


