cmake_minimum_required(VERSION 2.8.3)
project(rc_grasp)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

#generate_messages(DEPENDENCIES sensor_msgs)

SET(RW_ROOT "$ENV{RW_ROOT}")
catkin_package()

#Include default settings for constructing a robwork dependent project
FIND_PACKAGE(RobWork QUIET HINTS "${RW_ROOT}")
IF(NOT ROBWORK_FOUND)
  MESSAGE(STATUS "Could not find RobWork with the new method. Trying the old one.")
  INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
ENDIF(NOT ROBWORK_FOUND)

include_directories(${catkin_INCLUDE_DIRS} ${ROBWORK_INCLUDE_DIRS})
LINK_DIRECTORIES(${ROBWORK_LIBRARY_DIRS} ${catkin_LIBRARIES})

SET(SrcFiles src/graspNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")


