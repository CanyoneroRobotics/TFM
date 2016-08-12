cmake_minimum_required(VERSION 2.8.3)
project(canyonero_project)

find_package( catkin REQUIRED COMPONENTS 
      roscpp
      rospy
      tf
      std_msgs
      sensor_msgs)


include_directories(include ${catkin_INCLUDE_DIRS})
## The catkin_package macro generates cmake config files for your packagecatkin_package(

catkin_package(
  LIBRARIES 
  CATKIN_DEPENDS std_msgs roscpp rospy sensor_msgs tf
)

#source files
SET(SRCS 
      src/canyonero_control.cpp
      src/canyonero_odom.cpp)


add_executable(canyoneroControl src/canyonero_control.cpp)
add_dependencies(canyoneroControl ${catkin_EXPORTED_TARGETS})
target_link_libraries(canyoneroControl ${catkin_LIBRARIES})

add_executable(canyoneroOdometry src/canyonero_odom.cpp)
add_dependencies(canyoneroOdometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(canyoneroOdometry ${catkin_LIBRARIES})

