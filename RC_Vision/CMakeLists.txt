cmake_minimum_required(VERSION 2.8.3)
project(rc_vision)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation cv_bridge genmsg image_transport sensor_msgs OpenCV)

#add_message_files(FILES getBricks.srv)
add_service_files(FILES getBricks.srv)

#generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package()

# Use OpenCV
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

SET(SrcFiles src/visionNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")

