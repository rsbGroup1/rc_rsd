cmake_minimum_required(VERSION 2.8.3)
project(rsd_camera)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge genmsg image_transport sensor_msgs OpenCV)

#generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

# Use OpenCV
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

SET(SrcFiles src/cameraNode.cpp)

add_executable(rsd_camera_node ${SrcFiles})

target_link_libraries(rsd_camera_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")

