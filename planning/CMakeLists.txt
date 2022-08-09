cmake_minimum_required(VERSION 3.0.2)
project(planning)
SET(CMAKE_BUILD_TYPE Debug)
add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
)

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
  src/planning/math/aabox2d.cpp
  src/planning/math/box2d.cpp
  src/planning/math/line_segment2d.cpp
  src/planning/math/math_utils.cpp
  src/planning/math/polygon2d.cpp
  src/planning/math/vec2d.cpp
  src/planning/planning.cpp
  src/planning/discretized_trajectory.cpp
  src/planning/environment.cpp
  src/planning/dp_planner.cpp
  src/planning/trajectory_nlp.cpp
  src/planning/trajectory_optimizer.cpp
  src/planning/visualization/plot.cpp
  src/planning/visualization/color.cpp)

# add_library(${PROJECT_NAME} ${SRC})

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME} casadi)

add_executable(${PROJECT_NAME}_node src/planning_node.cpp ${SRC})

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
    casadi
)
