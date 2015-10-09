cmake_minimum_required(VERSION 2.8.3)
project(rc_camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS camera_info_manager image_transport nodelet roscpp sensor_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system)
set(V4L2_LIBRARIES v4l2)

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES nodelet_uvc_camera
   CATKIN_DEPENDS roscpp nodelet sensor_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(rc_camera src/cameraNode.cpp src/uvc_cam.cpp)
target_link_libraries(rc_camera ${Boost_LIBRARIES} ${V4L2_LIBRARIES} ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS rc_camera  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

