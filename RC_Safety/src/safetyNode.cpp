// Includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>

// Global variables
ros::Publisher _safetyPublisher, _hmiConsolePub;

// Functions
void printConsole(std::string msg)
{
    ROS_ERROR_STREAM(msg.c_str());
    std_msgs::String pubMsg;
    pubMsg.data = "Safety: " + msg;
    _hmiConsolePub.publish(pubMsg);
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "rc_safety");
    ros::NodeHandle nh;
    ros::NodeHandle pNh(ros::this_node::getName() + "/");

    // Topic names
    std::string safetyPub, hmiConsolePub;
    pNh.param<std::string>("safety_pub", safetyPub, "/rcSafety/status");
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");

    // Publishers
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);
    _safetyPublisher = nh.advertise<std_msgs::Bool>(safetyPub, 1);

    // Set loop rate
    ros::Rate loop_rate(10);

    while(nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Return
    return 0;
}
