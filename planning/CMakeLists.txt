cmake_minimum_required(VERSION 3.0.2)
project(planning)

SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_EXPORT_COMPILE_COMMANDS 1)
add_compile_options(-std=c++14 -O3)

# find ros package
find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_generation
        geometry_msgs
        )

# find other package
find_package(gflags REQUIRED)
find_package(osqp REQUIRED)

add_message_files(
        FILES
        CenterLinePoint.msg
        CenterLine.msg
        DynamicObstacle.msg
        DynamicObstacles.msg
        Obstacles.msg
        DynamicTrajectoryPoint.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
        # INCLUDE_DIRS include
        LIBRARIES
        ${PROJECT_NAME}

        CATKIN_DEPENDS
        roscpp
        message_runtime
)

include_directories(
        src
        ${catkin_INCLUDE_DIRS}
)

option(WITH_HSL "Compile solver with coinhsl" ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(HSL)
if (NOT ${HSL_FOUND})
    set(WITH_HSL OFF)
endif ()

if (WITH_HSL)
    add_definitions(-DWITH_HSL)
endif ()

set(MATH_SRC
        src/math/aabox2d.cpp
        src/math/box2d.cpp
        src/math/line_segment2d.cpp
        src/math/math_utils.cpp
        src/math/polygon2d.cpp
        src/math/vec2d.cpp
        src/math/path_matcher.cpp
        src/math/linear_interpolation.cpp
        src/math/cartesian_frenet_conversion.cpp
        src/math/curve1d/quartic_polynomial_curve1d.cpp
        src/math/curve1d/quintic_polynomial_curve1d.cpp)
add_library(MATH_LIB
        ${MATH_SRC})

set(LATTICE_SRC
        src/lattice/lattice_trajectory1d.cpp
        src/lattice/trajectory1d_generator.cpp
        src/lattice/path_time_graph.cpp
        src/lattice/end_condition_sampler.cc
        src/lattice/lateral_qp_optimizer.cc
        src/lattice/lateral_osqp_optimizer.cc)

set(SRC
        src/common/st_point.cpp
        src/common/st_boundary.cpp
        src/common/obstacle.cpp
        src/common/planning_gflags.cc
        src/visualization/plot.cpp
        src/visualization/color.cpp
        src/reference_line/reference_line.cpp
        src/reference_line/reference_line_provider.cpp
        src/planning/discretized_trajectory.cpp
        src/planning/frame.cpp
        src/planning/trajectory_nlp.cpp
        src/planning/trajectory_optimizer.cpp
        src/planner/cartesian_planner.cpp
        src/planner/lattice_planner.cpp
        src/planner/dp_planner.cpp
        src/on_lane_planning.cpp)

add_executable(${PROJECT_NAME}_node src/planning_node.cpp ${SRC} ${LATTICE_SRC})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
        MATH_LIB
        ${catkin_LIBRARIES}
        casadi
        gflags
        glog
        pthread
        osqp::osqp)
