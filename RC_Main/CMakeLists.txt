cmake_minimum_required(VERSION 2.8.3)
project(rc_main)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

# Service files
#add_service_files(
	#FILES
#)

#generate_messages(
#	DEPENDENCIES
#)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

SET(SrcFiles src/mainNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

MESSAGE(${PROJECT_NAME} "Done!")

