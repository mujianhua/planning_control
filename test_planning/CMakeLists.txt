cmake_minimum_required(VERSION 3.0.2)
project(test_planning)

SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        common_me
        roslib
)

find_package(gflags REQUIRED)

catkin_package(
        CATKIN_DEPENDS
        common_me
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# path optimizer
set(PATH_OPTIMIZER_SRC
        src/config/planning_flags.cpp
        src/data_struct/reference_path.cpp
        src/data_struct/reference_path_impl.cpp
        src/path_smoother/path_smoother.cpp
        src/tools/spline.cpp)
add_executable(test_path_optimizer src/test/test_path_optimizer.cpp ${PATH_OPTIMIZER_SRC})
target_link_libraries(test_path_optimizer ${catkin_LIBRARIES} glog gflags)

# planning node
set(SRC
        src/lattice_planner.cpp
        src/reference_line_info.cpp
        src/test/test_planning_node.cpp include/tools/spline.h src/tools/spline.cpp)

add_executable(planning_node ${SRC})
target_link_libraries(planning_node ${catkin_LIBRARIES})