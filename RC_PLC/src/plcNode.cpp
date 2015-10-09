// Includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <iostream>
#include <rc_plc/MoveConv.h>
#include <rc_plc/StopConv.h>
#include <rc_plc/StartConv.h>

// Global variables
ros::Publisher _hmiConsolePub;

// Functions
void printConsole(std::string msg)
{
    ROS_ERROR_STREAM(msg.c_str());
    std_msgs::String pubMsg;
    pubMsg.data = "PLC: " + msg;
    _hmiConsolePub.publish(pubMsg);
}

bool moveCallback(rc_plc::MoveConv::Request &req, rc_plc::MoveConv::Response &res)
{
    //
}

bool startCallback(rc_plc::StartConv::Request &req, rc_plc::StartConv::Response &res)
{
    //
}

bool stopCallback(rc_plc::StopConv::Request &req, rc_plc::StopConv::Response &res)
{
    //
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "rc_plc");
    ros::NodeHandle nh;
    ros::NodeHandle pNh(ros::this_node::getName() + "/");

    // Topic names
    std::string hmiConsolePub;
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");

    // Publishers
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);

    // Create service
    ros::ServiceServer serviceMove = nh.advertiseService("/rcPLC/MoveConv", moveCallback);
    ros::ServiceServer serviceStart = nh.advertiseService("/rcPLC/StartConv", startCallback);
    ros::ServiceServer serviceStop = nh.advertiseService("/rcPLC/StopConv", stopCallback);

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
