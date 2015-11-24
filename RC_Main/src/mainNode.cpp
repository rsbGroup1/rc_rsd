// Includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <rc_vision/getBricks.h>
#include <rc_grasp/grabBrick.h>
#include <rc_plc/MoveConv.h>
#include <rc_plc/StartConv.h>
#include <rc_plc/StopConv.h>
#include <rc_plc/ChangeDirection.h>
#include <kuka_rsi/getConfiguration.h>
#include <kuka_rsi/getIsMoving.h>
#include <kuka_rsi/getSafety.h>
#include <rc_mes_client/server.h>
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
ros::Publisher _hmiConsolePub, _mesMessagePub, _mainStatusPub;
ros::ServiceClient _serviceGrabBrick, _serviceGetBricks, _serviceMove, _serviceStart, _serviceStop, _serviceChangeDir, _serviceGetIsMoving, _serviceGetConf, _serviceGetSafety;
bool _run = false, _safety = false, _anyBricks = false, _mesOrder = false;
boost::mutex _runMutex, _safetyMutex, _anyBricksMutex, _qMutex, _orderMutex;
double _qIdle[6] = {1.34037, 0.696857, 0.158417, 0.418082, -0.736247, -0.531764};
bool _positionQIdle = false;
int _red = 0, _blue = 0, _yellow = 0;

// Functions
void printConsole(std::string msg, bool ros_print = false)
{
    if(ros_print)
        ROS_INFO_STREAM(msg.c_str());
    std_msgs::String pubMsg;
    pubMsg.data = "Main: " + msg;
    _hmiConsolePub.publish(pubMsg);
    std::cout << pubMsg.data << std::endl;
}

bool grabBrick(Brick brick)
{
    rc_grasp::grabBrick obj;
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
    rc_vision::getBricks obj;

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
    rc_plc::MoveConv obj;
    obj.request.duration = duration;
    obj.request.direction = dir;
    if(!_serviceMove.call(obj))
        printConsole("Failed to call the 'serviceMoveConveyer'");
}

void startConveyerBelt(bool dir = false)
{
    rc_plc::StartConv obj;
    obj.request.direction = dir;
    if(!_serviceStart.call(obj))
        printConsole("Failed to call the 'serviceStartConveyer'");
}

void stopConveyerBelt()
{
    rc_plc::StopConv obj;
    if(!_serviceStop.call(obj))
        printConsole("Failed to call the 'serviceStopConveyer'");
}

void changeDirConveyerBelt(bool dir) // true = reverse
{
    rc_plc::ChangeDirection obj;
    obj.request.direction = dir;
    if(!_serviceChangeDir.call(obj))
        printConsole("Failed to call the 'serviceChangeDirConveyer'");
}

bool isRobotMoving()
{
    kuka_rsi::getIsMoving obj;
    if(!_serviceGetIsMoving.call(obj))
        printConsole("Failed to call the 'serviceIsRobotMoving'");

    return obj.response.isMoving;
}

void anyBrickCallback(std_msgs::Bool msg)
{
    // Fetch robot position
    kuka_rsi::getConfiguration getQObj;

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

void mesRecCallback(rc_mes_client::server msg)
{
    if(msg.cell == 1)
    {
        boost::unique_lock<boost::mutex> lock(_orderMutex);
        _blue = msg.blue;
        _red = msg.red;
        _yellow = msg.yellow;
        _mesOrder = true;
    }
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
    bool run, anyBricks, safety, positionIdle, mesOrder;

    while(true)
    {
        try
        {
            // Get run
            _runMutex.lock();
            run = _run;
            _runMutex.unlock();

            // Get if mes order
            _orderMutex.lock();
            mesOrder = _mesOrder;
            _orderMutex.unlock();

            if(run && mesOrder)
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

                        // Check safety
                        _safetyMutex.lock();
                        safety = _safety;
                        _safetyMutex.unlock();

                        if(safety == false)
                        {
                            if(bricks.empty() == false)
                            {
                                // Filter and choose the correct bricks
                                _orderMutex.lock();
                                int brickToPick = -1;
                                for(unsigned int i = 0; i<bricks.size(); i++)
                                {
                                    if(bricks[i].color == "red" && _red > 0)
                                    {
                                        _red--;
                                        brickToPick = i;

                                        std_msgs::String msg;
                                        msg.data = "red";
                                        _mainStatusPub.publish(msg);

                                        printConsole("Picking up red brick..");

                                        break;
                                    }
                                    else if(bricks[i].color == "blue" && _blue > 0)
                                    {
                                        _blue--;
                                        brickToPick = i;

                                        std_msgs::String msg;
                                        msg.data = "blue";
                                        _mainStatusPub.publish(msg);

                                        printConsole("Picking up blue brick..");

                                        break;
                                    }
                                    else if(bricks[i].color == "yellow" && _yellow > 0)
                                    {
                                        _yellow--;
                                        brickToPick = i;

                                        std_msgs::String msg;
                                        msg.data = "yellow";
                                        _mainStatusPub.publish(msg);

                                        printConsole("Picking up yellow brick..");

                                        break;
                                    }
                                }
                                _orderMutex.unlock();

                                if(brickToPick >= 0)
                                {
                                    // Grab brick
                                    bricks[brickToPick].size = 0.014; // Default size of standard LEGO width
                                    if(grabBrick(bricks[brickToPick]) == false)
                                    {
                                        // Get run
                                        _runMutex.lock();
                                        _run = false;
                                        _runMutex.unlock();

                                        std_msgs::String msg;
                                        msg.data = "grabError";
                                        _mainStatusPub.publish(msg);

                                        printConsole("Grab error!", true);
                                    }
                                }
                                else
                                {
                                    if(!_red && !_blue && !_yellow)
                                    {
                                        // If no more bricks to pick
                                        printConsole("Clearing conveyer belt..");
                                        printConsole("Waiting for order!");

                                        // Tell MES Server
                                        mesSend("Ok");

                                        // Move conveyer so that remaining bricks can be removed
                                        moveCoveyerBelt(15);

                                        boost::unique_lock<boost::mutex> lock(_orderMutex);
                                        _mesOrder = false;
                                    }
                                    else
                                    {
                                        // Move slightly more forward and stop
                                        moveCoveyerBelt(1);
                                    }
                                }
                            }
                            else
                            {
                                // If no more bricks to pick
                                if(!_red && !_blue && !_yellow)
                                {
                                    printConsole("Clearing conveyer belt..");
                                    printConsole("Waiting for order!");
                                    // Move conveyer so that remaining bricks can be removed
                                    moveCoveyerBelt(15);

                                    boost::unique_lock<boost::mutex> lock(_orderMutex);
                                    _mesOrder = false;
                                }
                                // Bricks are to far from robot, move forward
                                else
                                {
                                    startConveyerBelt();
                                    conveyerRunning = true;
                                }
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
            else
            {
                // Not running

                // Set run
                conveyerRunning = false;
                waitForBrick = false;
                waitForIdle = false;
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
    ros::init(argc, argv, "RC_Main");
    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");

    // Topic names
    std::string hmiConsolePub, grabService, getBricksService, plcService, anyBricksSub, safetySub, mesPub, mesSub, hmiStatusSub;
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");
    pNh.param<std::string>("grabService", grabService, "/rcGrasp/grabBrick");
    pNh.param<std::string>("getBricksService", getBricksService, "/rcVision/getBricks");
    pNh.param<std::string>("plcService", plcService, "/rcPLC");
    pNh.param<std::string>("anyBricks_sub", anyBricksSub, "/rcVision/anyBricks");
    pNh.param<std::string>("hmi_status_sub", hmiStatusSub, "/rcHMI/status");
    pNh.param<std::string>("mesPub", mesPub, "/rcMESClient/msgToServer");
    pNh.param<std::string>("mesSub", mesSub, "/rcMESClient/msgFromServer");

    // Create service calls
    _serviceGrabBrick = nh.serviceClient<rc_grasp::grabBrick>(grabService);
    _serviceGetBricks = nh.serviceClient<rc_vision::getBricks>(getBricksService);
    _serviceMove = nh.serviceClient<rc_plc::MoveConv>(plcService + "/MoveConv");
    _serviceStart = nh.serviceClient<rc_plc::StartConv>(plcService + "/StartConv");
    _serviceStop = nh.serviceClient<rc_plc::StopConv>(plcService + "/StopConv");
    _serviceChangeDir = nh.serviceClient<rc_plc::ChangeDirection>(plcService + "/ChangeDirection");
    _serviceGetIsMoving = nh.serviceClient<kuka_rsi::getIsMoving>("/KukaNode/IsMoving");
    _serviceGetConf = nh.serviceClient<kuka_rsi::getConfiguration>("/KukaNode/GetConfiguration");
    _serviceGetSafety = nh.serviceClient<kuka_rsi::getSafety>("/KukaNode/GetSafety");

    // Publishers
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);
    _mesMessagePub = nh.advertise<std_msgs::String>(mesPub, 100);
    _mainStatusPub = nh.advertise<std_msgs::String>("/rcMain/status", 100);

    // Subscribers
    ros::Subscriber anyBrickSub = nh.subscribe(anyBricksSub, 10, anyBrickCallback);
    ros::Subscriber mesMessageSub = nh.subscribe(mesSub, 10, mesRecCallback);
    ros::Subscriber hmiStatusSubs = nh.subscribe(hmiStatusSub, 10, hmiStatusCallback);

    // Main handler thread
    boost::thread mainThread(mainHandlerThread);

    // Spin rate
    ros::Rate r(10); // 10 hz

    // Spin
    while(ros::ok())
    {
        // Get safety
        kuka_rsi::getSafety obj;
        _serviceGetSafety.call(obj);
        _safetyMutex.lock();
        _safety = obj.response.safetyBreached;
        _safetyMutex.unlock();

        ros::spinOnce();
        r.sleep();
    }

    // Return
   mainThread.interrupt();
    return 0;
}
