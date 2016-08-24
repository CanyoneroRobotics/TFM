cmake_minimum_required(VERSION 2.8.3)
project(canyonero_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface
  diagnostic_updater roslaunch roslint roscpp sensor_msgs)

catkin_package(
  INCLUDE_DIRS inc
  LIBRARIES serial
  CATKIN_DEPENDS hardware_interface roscpp sensor_msgs
)

include_directories(inc ${catkin_INCLUDE_DIRS})

add_executable(canyonero_node
  src/canyonero_hardware.cpp
  src/Arduino.cpp)

target_link_libraries(canyonero_node serial ${catkin_LIBRARIES})

roslint_cpp(
  src/canyonero_hardware.cpp
  src/Arduino.cpp
  inc/canyonero_hardware.h
  inc/Arduino.h
)

roslaunch_add_file_check(launch)

install(TARGETS canyonero_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY inc/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
