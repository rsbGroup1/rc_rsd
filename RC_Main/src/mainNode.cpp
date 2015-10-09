// Includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <rc_main/getBricks.h>
#include <rc_main/grabBrick.h>
#include <rc_main/MoveConv.h>
#include <rc_main/StartConv.h>
#include <rc_main/StopConv.h>

// Global variables
ros::Publisher _hmiConsolePub, _mesMessagePub;
ros::ServiceClient _serviceGrabBrick, _serviceGetBricks, _serviceMove, _serviceStart, _serviceStop;

// Functions
void printConsole(std::string msg)
{
    ROS_ERROR_STREAM(msg.c_str());
    std_msgs::String pubMsg;
    pubMsg.data = "Main: " + msg;
    _hmiConsolePub.publish(pubMsg);
}

bool grabBrick(double x, double y, double theta, double size)
{
    rc_main::grabBrick obj;
    obj.request.x = x;
    obj.request.y = y;
    obj.request.theta = theta;
    obj.request.size = size;

    if(!_serviceGrabBrick.call(obj))
        printConsole("Failed to call the 'serviceGrabBrick'");

    return obj.response.success;
}

rc_main::getBricks getBricks()
{
    rc_main::getBricks obj;
    if(!_serviceGetBricks.call(obj))
        printConsole("Failed to call the 'serviceGetBricks'");

    return obj;
}

void moveCoveyerBelt()
{
    rc_main::MoveConv obj;
    if(!_serviceMove.call(obj))
        printConsole("Failed to call the 'serviceMoveConveyer'");
}

void startConveyerBelt()
{
    rc_main::StartConv obj;
    if(!_serviceStart.call(obj))
        printConsole("Failed to call the 'serviceStartConveyer'");
}

void stopConveyerBelt()
{
    rc_main::StopConv obj;
    if(!_serviceStop.call(obj))
        printConsole("Failed to call the 'serviceStopConveyer'");
}

void anyBrickCallback(std_msgs::Bool msg)
{
    //
}

void safetyCallback(std_msgs::Bool msg)
{
    //
}

void mesRecCallback(std_msgs::String msg)
{

}

void mesSend(std::string sendMsg)
{
    std_msgs::String msg;
    msg.data = sendMsg;
    _mesMessagePub.publish(msg);
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "rc_main");
    ros::NodeHandle nh;
    ros::NodeHandle pNh(ros::this_node::getName() + "/");

    // Topic names
    std::string hmiConsolePub, grabService, getBricksService, plcService, anyBricksSub, safetySub, mesPub, mesSub;
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");
    pNh.param<std::string>("grabService", grabService, "/rcGrasp/grabBrick");
    pNh.param<std::string>("getBricksService", getBricksService, "/rcVision/getBricks");
    pNh.param<std::string>("plcService", plcService, "/rcPLC");
    pNh.param<std::string>("anyBricks_sub", anyBricksSub, "/rcVision/anyBricks");
    pNh.param<std::string>("safety_sub", safetySub, "/rcSafety/status");
    pNh.param<std::string>("mesPub", mesPub, "/rcMESServer/msgToServer");
    pNh.param<std::string>("mesSub", mesSub, "/rcMESServer/msgFromServer");

    // Create service calls
    _serviceGrabBrick = nh.serviceClient<rc_main::grabBrick>(grabService);
    _serviceGetBricks = nh.serviceClient<rc_main::getBricks>(getBricksService);
    _serviceMove = nh.serviceClient<rc_main::MoveConv>(plcService + "/MoveConv");
    _serviceStart = nh.serviceClient<rc_main::StartConv>(plcService + "/StartConv");
    _serviceStop = nh.serviceClient<rc_main::StopConv>(plcService + "/StopConv");

    // Publishers
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);
    _mesMessagePub = nh.advertise<std_msgs::String>(mesPub, 100);

    // Subscribers
    ros::Subscriber anyBrickSub = nh.subscribe(anyBricksSub, 10, anyBrickCallback);
    ros::Subscriber safetySubs = nh.subscribe(safetySub, 10, safetyCallback);
    ros::Subscriber mesMessageSub = nh.subscribe(mesSub, 10, mesRecCallback);

    // Set loop rate
    ros::Rate loop_rate(10);

    while(nh.ok())
    {
        std::cout << "Ok" << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Return
    return 0;
}
