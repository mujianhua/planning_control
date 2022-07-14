cmake_minimum_required(VERSION 3.0.2)
project(test_control)
SET(CMAKE_BUILD_TYPE Debug)

# 找ros自己的库
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    message_generation
)
# 找其他库
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)


add_message_files(
    FILES
    chassis_cmd.msg
    chassis_data.msg
    simple_mpc_debug.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    roscpp
    rospy
    std_msgs)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
add_definitions(${EIGEN_DEFINITIONS})

set(SRC 
    src/test_control_node.cpp
    src/mpc_controller.cpp
    src/ADCTrajectory.cpp
    src/trajectory_analyzer.cpp
    src/vehicle_state_provider.cpp
    src/math/mpc_osqp.cpp)
add_executable(test_control ${SRC})
target_link_libraries(test_control ${catkin_LIBRARIES} osqp::osqp)

add_executable(test_pub_node src/test_pub_node.cpp)
target_link_libraries(test_pub_node ${catkin_LIBRARIES})

