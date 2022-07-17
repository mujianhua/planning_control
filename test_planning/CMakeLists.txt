cmake_minimum_required(VERSION 3.0.2)
project(test_planning)

SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    test_common
    roslib
)

find_package(gflags REQUIRED)

catkin_package(
    CATKIN_DEPENDS 
    test_common
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# path optimizer
set(PATH_OPTIMIZER_SRC
    src/config/planning_flags.cpp)
add_executable(test_path_optimizer src/test/test_path_optimizer.cpp ${PATH_OPTIMIZER_SRC})
target_link_libraries(test_path_optimizer ${catkin_LIBRARIES} glog gflags)

# planning node
set(SRC
    src/lattice_planner.cpp
    src/reference_line_info.cpp
    src/test/test_planning_node.cpp)

add_executable(planning_node ${SRC})
target_link_libraries(planning_node ${catkin_LIBRARIES})