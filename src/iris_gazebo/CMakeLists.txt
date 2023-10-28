cmake_minimum_required(VERSION 3.0.2)
project(iris_gazebo)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES fusion2urdf
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(robot_transmission SHARED src/robot_transmission.cpp)
target_link_libraries(robot_transmission ${GAZEBO_LIBRARIES})

catkin_install_python(PROGRAMS scripts/twist_3_omni.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)