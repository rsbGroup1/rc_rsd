// Includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>

// Global variables
ros::Publisher _hmiConsolePub, _mesMessagePub;
std::string _serverIP;
int _serverPort;

// Functions
void printConsole(std::string msg)
{
    ROS_ERROR_STREAM(msg.c_str());
    std_msgs::String pubMsg;
    pubMsg.data = "MES: " + msg;
    _hmiConsolePub.publish(pubMsg);
}

void sendMsgCallback(std_msgs::String msg)
{
    // Construct and send message to server
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "rc_messerver");
    ros::NodeHandle nh;
    ros::NodeHandle pNh(ros::this_node::getName() + "/");

    // Topic names
    std::string hmiConsolePub, mesSub, mesPub;
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");
    pNh.param<std::string>("mesPub", mesPub, "/rcMESServer/msgFromServer");
    pNh.param<std::string>("mesSub", mesSub, "/rcMESServer/msgToServer");
    pNh.param<std::string>("server_ip", _serverIP, "192.168.100.124");
    pNh.param<int>("server_port", _serverPort, 14141);

    // Publishers
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);
    _mesMessagePub = nh.advertise<std_msgs::String>(mesPub, 100);

    // Subscribers
    ros::Subscriber mesMessageSub = nh.subscribe(mesSub, 10, sendMsgCallback);

    // Set loop rate
    while(ros::ok())
        ros::spinOnce();

    // Return
    return 0;
}
