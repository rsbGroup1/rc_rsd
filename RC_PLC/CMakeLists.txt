cmake_minimum_required(VERSION 2.8.3)
project(rc_plc)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg serial)


# Service files
add_service_files(
	FILES
	StopConv.srv
	StartConv.srv
	MoveConv.srv
	ChangeDirection.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()


include_directories(${catkin_INCLUDE_DIRS})

SET(SrcFiles src/plcNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")

