cmake_minimum_required(VERSION 3.0.2)
project(serial_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_demo
#  CATKIN_DEPENDS roscpp serial
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_demo src/serial_demo.cpp)
 
add_dependencies(serial_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 
target_link_libraries(serial_demo
  ${catkin_LIBRARIES}
)
