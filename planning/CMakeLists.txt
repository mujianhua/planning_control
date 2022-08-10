cmake_minimum_required(VERSION 3.0.2)
project(planning)
SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_EXPORT_COMPILE_COMMANDS 1)
add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
)

find_package(gflags REQUIRED)

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
  INCLUDE_DIRS include
  LIBRARIES planning
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

option(WITH_HSL "Compile solver with coinhsl" ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(HSL)
if(NOT ${HSL_FOUND})
  set(WITH_HSL OFF)
endif()

if(WITH_HSL)
  add_definitions(-DWITH_HSL)
endif()

set(SRC
  src/math/aabox2d.cpp
  src/math/box2d.cpp
  src/math/line_segment2d.cpp
  src/math/math_utils.cpp
  src/math/polygon2d.cpp
  src/math/vec2d.cpp
  src/math/cartesian_frenet_conversion.cpp
  src/visualization/plot.cpp
  src/visualization/color.cpp
  src/planning/planning_config.cpp
  src/planning/reference_line.cpp
  src/planning/discretized_trajectory.cpp
  src/planning/frame.cpp
  src/planning/trajectory_nlp.cpp
  src/planning/trajectory_optimizer.cpp
  src/planner/cartesian_planner.cpp
  src/planner/dp_planner.cpp)

add_executable(${PROJECT_NAME}_node src/planning_node.cpp ${SRC})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
    casadi
    gflags
)
