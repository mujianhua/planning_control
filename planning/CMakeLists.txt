cmake_minimum_required(VERSION 3.0.2)
project(planning)

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
        CATKIN_DEPENDS roscpp message_runtime
        #  LIBRARIES planning
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(SRC
        src/common/frame.cpp
        src/common/obstacle.cpp
        src/common/math/math_utils.cpp
        src/common/math/aabox2d.cpp
        src/common/math/box2d.cpp
        src/common/math/line_segment2d.cpp
        src/common/math/polygon2d.cpp
        src/common/math/vec2d.cpp
        src/reference_line/reference_line.cpp
        src/reference_line/reference_line_provider.cpp
        src/planner/cartesian_planner.cpp
        src/planning_base.cpp
        src/on_lane_planning.cpp)

add_executable(test_${PROJECT_NAME} src/test_planning_node.cpp ${SRC})
target_link_libraries(test_${PROJECT_NAME} ${catkin_LIBRARIES})
