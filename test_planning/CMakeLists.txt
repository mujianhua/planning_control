cmake_minimum_required(VERSION 3.0.2)
project(test_planning)

SET(CMAKE_BUILD_TYPE Debug)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(catkin_deps
        roscpp
        common_me
        grid_map_ros
        ros_viz_tools
        tinyspline_ros)

find_package(catkin REQUIRED COMPONENTS
        ${catkin_deps}
        rospy
        std_msgs
        roslib
        )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
message(STATUS "CMAKE SOURCE DIR:" ${CMAKE_MODULE_PATH})

find_package(IPOPT REQUIRED)
find_package(OsqpEigen REQUIRED)
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
        src/path_optimizer/path_optimizer.cpp
        src/path_optimizer/qp_path_optimizer.cpp
        src/common/frame.cpp
        src/common/data_struct.cpp
        src/common/spline.cpp
        src/common/vehicle_state.cpp
        src/common/math/math_util.cpp
        src/tools/map.cpp
        src/reference_line/reference_point.cpp
        src/reference_line/reference_line.cpp
        src/reference_line/qp_spline_reference_smoother.cpp)
        
add_executable(test_path_optimizer src/test/test_path_optimizer.cpp ${PATH_OPTIMIZER_SRC})
target_link_libraries(test_path_optimizer ${catkin_LIBRARIES} glog gflags ${OpenCV_LIBS} ${IPOPT_LIBRARIES}
        OsqpEigen::OsqpEigen osqp::osqp)

# planning node
set(SRC
        src/lattice_planner.cpp
        src/reference_line_info.cpp
        src/test/test_planning_node.cpp)

add_executable(planning_node ${SRC})
target_link_libraries(planning_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
        find_package(rostest)
        catkin_add_gtest(cartesian_frenet_conversion_test
                test/cartesian_frenet_conversion_test.cpp
                src/common/math/cartesian_frenet_conversion.cpp)
endif()