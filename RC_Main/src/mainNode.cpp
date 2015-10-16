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

// Defines
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Global variables
ros::Publisher _hmiConsolePub, _mesMessagePub;
ros::ServiceClient _serviceGrabBrick, _serviceGetBricks, _serviceMove, _serviceStart, _serviceStop;
bool _run = false, _safety = false;

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
    static bool anyBricks = false;

    if(anyBricks == false)
    {
        // Get positions etc.
        rc_main::getBricks bricks;
        if(!_serviceGetBricks.call(bricks))
            printConsole("Failed to call the 'serviceGrabBricks'");

        // Grab them
        for(unsigned int i=0; i<1; i++) // bricks.response.color.size()
        {
            // x, y, theta, size
            grabBrick(bricks.response.posX[i], bricks.response.posY[i], bricks.response.theta[i], 0.016);
        }
    }

    //anyBricks = true;
}

void safetyCallback(std_msgs::Bool msg)
{
    _safety = msg.data;
}

void mesRecCallback(std_msgs::String msg)
{
    // Store order
}

void mesSend(std::string sendMsg)
{
    std_msgs::String msg;
    msg.data = sendMsg;
    _mesMessagePub.publish(msg);
}

void hmiStatusCallback(std_msgs::String msg)
{
    if(msg.data == "start")
        _run = true;
    else if(msg.data == "stop")
        _run = false;
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
    std::string hmiConsolePub, grabService, getBricksService, plcService, anyBricksSub, safetySub, mesPub, mesSub, hmiStatusSub;
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");
    pNh.param<std::string>("grabService", grabService, "/rcGrasp/grabBrick");
    pNh.param<std::string>("getBricksService", getBricksService, "/rcVision/getBricks");
    pNh.param<std::string>("plcService", plcService, "/rcPLC");
    pNh.param<std::string>("anyBricks_sub", anyBricksSub, "/rcVision/anyBricks");
    pNh.param<std::string>("safety_sub", safetySub, "/rcSafety/status");
    pNh.param<std::string>("hmi_status_sub", hmiStatusSub, "/rcHMI/status");
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
    ros::Subscriber hmiStatusSubs = nh.subscribe(hmiStatusSub, 10, hmiStatusCallback);

    // Set loop rate
    ros::Rate loop_rate(10);

    // Main loop
    while(nh.ok())
    {
        // If "HMI: Start", "Safety: False" and "MES: Order"
        // Start conveyer
        // If "anyBricks: True"
        // Stop conveyer
        // Move conveyer
        // Call getBricks
        // Look which bricks to pick up (compare to MES order)
            // Grab one brick and if more bricks matched the order, call getBricks again in case of bricks that has moved
        // Move conveyer
        // If getBricks = 0, move more


        //std::cout << "Ok" << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Return
    return 0;
}
