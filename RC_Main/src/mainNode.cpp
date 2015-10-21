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
#include <rc_main/ChangeDirection.h>
#include <rc_main/getIsMoving.h>
#include <rc_main/getConfiguration.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

// Defines
#define DEGREETORAD             (M_PI/180.0)
#define RADTODEGREE             (180.0/M_PI)
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Brick struct
struct Brick
{
    std::string color;
    float size;
    float posX;
    float posY;
    float theta;
};

// Global variables
ros::Publisher _hmiConsolePub, _mesMessagePub;
ros::ServiceClient _serviceGrabBrick, _serviceGetBricks, _serviceMove, _serviceStart, _serviceStop, _serviceChangeDir, _serviceGetIsMoving, _serviceGetConf;
bool _run = false, _safety = false, _anyBricks = false;
boost::mutex _runMutex, _safetyMutex, _anyBricksMutex, _qMutex;
double _qIdle[6] = {1.34037, 0.696857, 0.158417, 0.418082, -0.736247, -0.531764};
bool _positionQIdle = false;

// Functions
void printConsole(std::string msg)
{
    ROS_ERROR_STREAM(msg.c_str());
    std_msgs::String pubMsg;
    pubMsg.data = "Main: " + msg;
    _hmiConsolePub.publish(pubMsg);
}

bool grabBrick(Brick brick)
{
    rc_main::grabBrick obj;
    obj.request.x = brick.posX;
    obj.request.y = brick.posY;
    obj.request.theta = brick.theta;
    obj.request.size = brick.size;

    if(!_serviceGrabBrick.call(obj))
        printConsole("Failed to call the 'serviceGrabBrick'");

    return obj.response.success;
}

std::vector<Brick> getBricks()
{
    std::vector<Brick> retVec;
    rc_main::getBricks obj;

    if(!_serviceGetBricks.call(obj))
    {
        printConsole("Failed to call the 'serviceGetBricks'");
    }
    else
    {
        for(unsigned int i=0; i<obj.response.size.size(); i++)
        {
            Brick brick;
            brick.color = obj.response.color[i];
            brick.posX = obj.response.posX[i];
            brick.posY = obj.response.posY[i];
            brick.size = obj.response.size[i];
            brick.theta = obj.response.theta[i];
            retVec.push_back(brick);
        }
    }

    return retVec;
}

void moveCoveyerBelt(double duration, bool dir = false)
{
    rc_main::MoveConv obj;
    obj.request.duration = duration;
    obj.request.direction = dir;
    if(!_serviceMove.call(obj))
        printConsole("Failed to call the 'serviceMoveConveyer'");
}

void startConveyerBelt(bool dir = false)
{
    rc_main::StartConv obj;
    obj.request.direction = dir;
    if(!_serviceStart.call(obj))
        printConsole("Failed to call the 'serviceStartConveyer'");
}

void stopConveyerBelt()
{
    rc_main::StopConv obj;
    if(!_serviceStop.call(obj))
        printConsole("Failed to call the 'serviceStopConveyer'");
}

void changeDirConveyerBelt(bool dir) // true = reverse
{
    rc_main::ChangeDirection obj;
    obj.request.direction = dir;
    if(!_serviceChangeDir.call(obj))
        printConsole("Failed to call the 'serviceChangeDirConveyer'");
}

bool isRobotMoving()
{
    rc_main::getIsMoving obj;
    if(!_serviceGetIsMoving.call(obj))
        printConsole("Failed to call the 'serviceIsRobotMoving'");

    return obj.response.isMoving;
}

void anyBrickCallback(std_msgs::Bool msg)
{
    // Fetch robot position
    rc_main::getConfiguration getQObj;

    // Call service
    if(!_serviceGetConf.call(getQObj))
       printConsole("Failed to call the 'serviceKukaGetConfiguration'");

    // Get information
    bool same = true;
    for(int i=0; i<6; i++)
    {
        if(fabs(getQObj.response.q[i]*DEGREETORAD - _qIdle[i]) > 0.1)
        {
            same = false;
            break;
        }
    }

    _anyBricksMutex.lock();
    _anyBricks = msg.data;
    _anyBricksMutex.unlock();

    _qMutex.lock();
    _positionQIdle = same;
    _qMutex.unlock();
}

void safetyCallback(std_msgs::Bool msg)
{
    boost::unique_lock<boost::mutex> lock(_safetyMutex);
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
    boost::unique_lock<boost::mutex> lock(_runMutex);
    if(msg.data == "start")
        _run = true;
    else if(msg.data == "stop")
        _run = false;
}

void mainHandlerThread()
{
    bool waitForBrick = false;
    bool waitForIdle = false;
    bool conveyerRunning = false;
    bool run, anyBricks, safety, positionIdle;

    while(true)
    {
        try
        {
            // Get run
            _runMutex.lock();
            run = _run;
            _runMutex.unlock();

            if(run)
            {
                // Get position
                _qMutex.lock();
                positionIdle = _positionQIdle;
                _qMutex.unlock();

                // Check if in idle position
                if(positionIdle)
                {
                    waitForIdle == false;

                    // Get anyBricks
                    _anyBricksMutex.lock();
                    anyBricks = _anyBricks;
                    _anyBricksMutex.unlock();

                    // Check if bricks
                    if(anyBricks)
                    {
                        waitForBrick = false;

                        // Stop conveyer belt
                        if(conveyerRunning)
                        {
                            // Move slightly more forward and stop
                            moveCoveyerBelt(1);
                            //stopConveyerBelt();
                            conveyerRunning = false;
                        }

                        // Wait til robot is not moving
                        if(isRobotMoving())
                            continue;

                        // Sleep in order for image to settle
                        sleep(3);

                        // Get bricks
                        std::vector<Brick> bricks = getBricks();

                        // Filter and choose the correct bricks
                        // ..

                        // Check safety
                        _safetyMutex.lock();
                        safety = _safety;
                        _safetyMutex.unlock();

                        if(safety == false)
                        {
                            if(bricks.size())
                            {
                                // Grab first brick for now
                                bricks.front().size = 0.015; // Default size of standard LEGO width
                                grabBrick(bricks.front());
                            }
                            else // Bricks are to far from robot, move forward
                            {
                                conveyerRunning = true;
                            }
                        }
                        else
                        {
                            // Get run
                            _runMutex.lock();
                            _run = false;
                            _runMutex.unlock();
                        }
                    }
                    else
                    {
                        // Start conveyer if not already running
                        if(conveyerRunning == false)
                        {
                            startConveyerBelt();
                            conveyerRunning = true;
                        }

                        // Inform user
                        if(waitForBrick == false)
                        {
                            printConsole("Waiting for bricks!");
                            waitForBrick = true;
                        }
                    }
                }
                else
                {
                    // Inform user
                    if(waitForIdle == false)
                    {
                        printConsole("Waiting for idle position!");
                        waitForIdle = true;
                    }
                }
            }

            // Signal interrupt point and sleep
            boost::this_thread::interruption_point();
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
        catch(const boost::thread_interrupted&)
        {
            break;
        }
    }
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
    _serviceChangeDir = nh.serviceClient<rc_main::ChangeDirection>(plcService + "/ChangeDirection");
    _serviceGetIsMoving = nh.serviceClient<rc_main::getIsMoving>("/KukaNode/IsMoving");
    _serviceGetConf = nh.serviceClient<rc_main::getConfiguration>("/KukaNode/GetConfiguration");

    // Publishers
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);
    _mesMessagePub = nh.advertise<std_msgs::String>(mesPub, 100);

    // Subscribers
    ros::Subscriber anyBrickSub = nh.subscribe(anyBricksSub, 10, anyBrickCallback);
    ros::Subscriber safetySubs = nh.subscribe(safetySub, 10, safetyCallback);
    ros::Subscriber mesMessageSub = nh.subscribe(mesSub, 10, mesRecCallback);
    ros::Subscriber hmiStatusSubs = nh.subscribe(hmiStatusSub, 10, hmiStatusCallback);

    // Main handler thread
    boost::thread mainThread(mainHandlerThread);

    // Spin
   ros::spin();

    // Return
   mainThread.interrupt();
    return 0;
}
