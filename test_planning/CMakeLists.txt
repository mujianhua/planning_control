cmake_minimum_required(VERSION 3.0.2)
project(test_planning)

SET(CMAKE_BUILD_TYPE Debug)

set(catkin_deps
        roscpp
        common_me
        grid_map_ros
        ros_viz_tools)

find_package(catkin REQUIRED COMPONENTS
        ${catkin_deps}
        rospy
        std_msgs
        roslib
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)

catkin_package(
        CATKIN_DEPENDS ${catkin_deps}
        DEPENDS OpenCV
)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

# path optimizer
set(PATH_OPTIMIZER_SRC
        src/config/planning_flags.cpp
        src/data_struct/reference_path.cpp
        src/data_struct/reference_path_impl.cpp
        src/path_optimizer/path_optimizer.cpp
        src/path_smoother/reference_path_smoother.cpp
        src/tools/spline.cpp
        src/tools/vehicle_state.cpp
        src/math/math_util.cpp)
add_executable(test_path_optimizer src/test/test_path_optimizer.cpp ${PATH_OPTIMIZER_SRC})
target_link_libraries(test_path_optimizer ${catkin_LIBRARIES} glog gflags ${OpenCV_LIBS})

# planning node
set(SRC
        src/lattice_planner.cpp
        src/reference_line_info.cpp
        src/test/test_planning_node.cpp include/tools/spline.h src/tools/spline.cpp)

add_executable(planning_node ${SRC})
target_link_libraries(planning_node ${catkin_LIBRARIES})