cmake_minimum_required(VERSION 3.0.2)
project(test_planning)

SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    test_common
)


# catkin_package(
#     LIBRARIES test_planning
#     CATKIN_DEPENDS roscpp rospy std_msgs
#     test_common
# )

catkin_package(
    CATKIN_DEPENDS 
    test_common
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