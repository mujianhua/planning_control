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
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_demo src/serial_demo.cpp)
 
add_dependencies(serial_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 
target_link_libraries(serial_demo
  ${catkin_LIBRARIES}
)

set (SRC
      src/wit_serial.cpp)
add_executable(test_serial src/test_serial.cpp ${SRC})
 
add_dependencies(test_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 
target_link_libraries(test_serial
  ${catkin_LIBRARIES}
)
