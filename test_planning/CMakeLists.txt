cmake_minimum_required(VERSION 3.0.2)
project(test_planning)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    test_common
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_planning
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(SRC
    src/lattice_planner.cpp
    src/reference_line_info.cpp
    src/test_planning_node.cpp)

add_executable(test_planning ${SRC})
target_link_libraries(test_planning ${catkin_LIBRARIES})