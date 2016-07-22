cmake_minimum_required(VERSION 2.8.3)
project(joystick_control)

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
      src/velocity_control.cpp
      src/velocity_control_double.cpp)


add_executable(joystick_control_simple src/velocity_control.cpp)
add_dependencies(joystick_control_simple ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_control_simple ${catkin_LIBRARIES})

add_executable(joystick_control_double src/velocity_control_double.cpp)
add_dependencies(joystick_control_double ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_control_double ${catkin_LIBRARIES})

