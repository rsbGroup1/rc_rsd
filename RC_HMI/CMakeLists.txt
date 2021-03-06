# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(rc_hmi)

## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation cv_bridge genmsg image_transport sensor_msgs)

## Declare ROS messages and services
# Service files
#add_service_files(
	#FILES
#)

# Use OpenCV
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

## Generate added messages and services with any dependencies listed here
#generate_messages(
	#DEPENDENCIES
	#std_msgs  # Or other packages containing msgs
#)

catkin_package(CATKIN_DEPENDS message_runtime)

# We use the settings that robwork studio uses
SET(CMAKE_BUILD_TYPE Release)
SET(RW_ROOT "$ENV{RW_ROOT}")
SET(RWSTUDIO_ROOT "$ENV{RWS_ROOT}")

#Include default settings for constructing a robwork dependent project
FIND_PACKAGE(RobWork QUIET HINTS "${RW_ROOT}")
IF(NOT ROBWORK_FOUND)
  MESSAGE(STATUS "Could not find RobWork with the new method. Trying the old one.")
  INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
ENDIF(NOT ROBWORK_FOUND)

#Include default settings for constructing a robworkst dependent project
FIND_PACKAGE(RobWorkStudio QUIET HINTS "${RWSTUDIO_ROOT}")
IF(NOT ROBWORKSTUDIO_FOUND)
  MESSAGE(STATUS "Could not find RobWorkStudio with the new method. Trying the old one.")
  INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)
ENDIF(NOT ROBWORKSTUDIO_FOUND)

# Include and link
INCLUDE_DIRECTORIES(${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} ${catkin_LIBRARIES} ${OpenCV_LIB_DIR})

# If we want to use ui files add them here
SET(UIS_FILES src/HMIWidget.ui)
SET(SrcFiles src/HMIPlugin.cpp src/HMIWidget.cpp ${UIS_OUT_H})
SET(ROS_BUILD_STATIC_LIBS false)
SET(ROS_BUILD_SHARED_LIBS true)

QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
QT4_WRAP_CPP(MocSrcFiles src/HMIPlugin.hpp src/HMIWidget.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
QT4_ADD_RESOURCES(RccSrcFiles src/resources.qrc)

ADD_LIBRARY(${PROJECT_NAME} ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) 
