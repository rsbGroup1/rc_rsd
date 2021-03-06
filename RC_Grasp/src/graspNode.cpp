// Includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <rw/rw.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/math.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <kuka_rsi/setConfiguration.h>
#include <kuka_rsi/getConfiguration.h>
#include <kuka_rsi/stopRobot.h>
#include <kuka_rsi/getIsMoving.h>
#include <kuka_rsi/getQueueSize.h>
#include <pg70/Move.h>
#include <pg70/Stop.h>
#include <pg70/Open.h>
#include <rc_grasp/grabBrick.h>
#include <boost/thread/mutex.hpp>

#include <iostream>

// Defines
#define SSTR(x)                     dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()
#define DEGREETORAD                 (M_PI/180.0)
#define RADTODEGREE                 (180.0/M_PI)
#define ROBOT_NAME                  "KukaKR6R700"
#define CONNECT_KUKA                true
#define CONNECT_PG70                true
#define PITCH_OFFSET                (-18*DEGREETORAD)
#define GRIPPER_MAX                 0.034
#define SPEED                       100

// Global variables
ros::ServiceClient _serviceKukaSetConf, _serviceKukaStop, _serviceKukaGetConf, _serviceKukaGetQueueSize, _serviceKukaGetIsMoving;
ros::ServiceClient _servicePG70Move, _servicePG70Stop, _servicePG70Open;
rw::models::WorkCell::Ptr _workcell;
rw::models::Device::Ptr _device;
rw::kinematics::State _state;
rw::invkin::JacobianIKSolver *_inverseKinGripper, *_inverseKinCamera;
rw::kinematics::Frame *_gripperFrame, *_cameraFrame, *_brickFrame;
rw::math::Transform3D<> _w2base, _w2camera, _w2brick;
double _idleQHeight, _xMax, _yMax, _graspOffset, _graspLifted, _gripperOpenOffset;
rw::math::Q _idleQ, _releaseBrickQ;
rw::models::Device::QBox _limits;
ros::Publisher _hmiConsolePub;
boost::mutex _runMutex;
bool _run;

// Prototypes
void KukaStopRobot();
bool KukaIsMoving();
bool KukaSetConf(rw::math::Q q, rw::math::Q speed = rw::math::Q(6,0,0,0,0,0,0));
int KukaGetQueueSize();
rw::math::Q KukaGetConf();
bool PG70SetConf(rw::math::Q q);
void PG70Stop();
void PG70Open();

// Functions
bool moveRobotWait(rw::math::Q q, rw::math::Q speed = rw::math::Q(6,0,0,0,0,0,0))
{
    if(KukaSetConf(q, speed))
    {
        while(KukaIsMoving())
            usleep(100);

        return true;
    }
    else
        return false;
}

void printConsole(std::string msg)
{
    //ROS_INFO_STREAM(msg.c_str());
    std_msgs::String pubMsg;
    pubMsg.data = "Grasp: " + msg;
    _hmiConsolePub.publish(pubMsg);
}

void printT(rw::math::Transform3D<> T)
{
    rw::math::RPY<> rpy(T.R());
    std::cout << "P(" << T.P()[0] << "," << T.P()[1] << "," << T.P()[2] << "), RPY(" << rpy[0]*RADTODEGREE << "," << rpy[1]*RADTODEGREE << "," << rpy[2]*RADTODEGREE << ")" << std::endl;
}

rw::math::Q getQFromPinBrickFrame(rw::math::Vector3D<> p, double rotation)
{
    // Configuration for lego brick
    rw::math::Q qRet(6,0,0,0,0,0,0);

    // Inverse kin
    rw::math::Transform3D<> posOffset(p);
    rw::math::Transform3D<> w2brickOffset = _w2brick * posOffset;
    rw::math::Transform3D<> transform((inverse(_w2base)*w2brickOffset).P(), rw::math::RPY<>(M_PI, PITCH_OFFSET, M_PI));
    std::vector<rw::math::Q> qVec = _inverseKinGripper->solve(transform, _state);
    if(qVec.empty())
    {
        // Log
        printConsole("Pick up frame: Error in inverse kinematics!");
        return qRet;
    }

    qRet = qVec[0];

    // Fix rotation
    if(qRet[5] + rotation < _limits.first[5])
        // Log
        printConsole("Joint 5 reached its limit!");
    else if(qRet[5] + rotation > _limits.second[5])
        // Log
        printConsole("Joint 5 reached its limit!");
    else
        qRet[5] += rotation;

    return qRet;
}

bool checkQ(rw::math::Q q)
{
    unsigned int counter = 0;
    for(unsigned int i=0; i<q.size(); i++)
        if(q(i) <= 0.001 && q(i) >= -0.001)
            counter++;

    if(counter == q.size())
        return false;
    else
        return true;
}

bool getStatus(void)
{
    ros::spinOnce();
    boost::unique_lock<boost::mutex> lock(_runMutex);
    return _run;
    //return true;
}

void hmiStatusCallback(std_msgs::String msg)
{
    boost::unique_lock<boost::mutex> lock(_runMutex);
    if(msg.data == "start")
        _run = true;
    else if(msg.data == "stop")
        _run = false;
}

rw::math::Q getSpeed(double speed)
{
    rw::math::Q q(6);

    for(unsigned int i=0; i<q.size(); i++)
        q(i) = speed;

    return q;
}

bool grabBrickCallback(rc_grasp::grabBrick::Request &req, rc_grasp::grabBrick::Response &res)
{
    // Check limits
    if(req.x>_xMax || req.x<-_xMax || req.y>_yMax || req.y<-_yMax)
    {
        // Log
        printConsole("Position larger than workspace!");
        return false;
    }
    if(req.size>GRIPPER_MAX)
    {
        // Log
        printConsole("Lego size larger than workspace!");
        return false;
    }

    // Default return message
    res.success = false;

    // Configuration for lego brick
    rw::math::Q qBrickLifted = getQFromPinBrickFrame(rw::math::Vector3D<>(req.x, req.y, _graspLifted), req.theta);
    rw::math::Q qBrick = getQFromPinBrickFrame(rw::math::Vector3D<>(req.x, req.y, _graspOffset), req.theta);

    if(checkQ(qBrickLifted) == false || checkQ(qBrick) == false)
        return false;

    // Log
    printConsole("Opening gripper!");

    // 1. Open gripper
    // Gripper max cmd: 0.034m. Equals an opening of 0.068m. If 5cm opening is wanted: Send 0.025
    if(!getStatus())
        return false;
    rw::math::Q qGripperOpen(1);
    if((req.size+_gripperOpenOffset)/2.0 > GRIPPER_MAX)
        qGripperOpen(0) = GRIPPER_MAX;
    else
        qGripperOpen(0) = (req.size+_gripperOpenOffset)/2.0;
    if(PG70SetConf(qGripperOpen) == false)
        return false;

    // Log
    printConsole("Moving robot to just above brick!");

    // 2. Move to brick lifted (blocking call)
    if(!getStatus())
        return false;
    rw::math::Q speed = getSpeed(SPEED);
    if(moveRobotWait(qBrickLifted, speed) == false)
        return false;

    // Log
    printConsole("Moving robot to grasp brick (x,y,theta) (" + SSTR(req.x) + "," + SSTR(req.y) + "," + SSTR(req.theta) + ")");

    // 3. Move to brick down (blocking call)
    if(!getStatus())
        return false;
    speed = getSpeed(10);
    if(moveRobotWait(qBrick, speed) == false)
        return false;

    // Log
    printConsole("Grasping brick!");

    // 4. Close gripper to req.size
    if(!getStatus())
        return false;
    rw::math::Q qGripperClose(1, req.size/2.0);
    if(PG70SetConf(qGripperClose) == false)
        return false;
    //sleep(1);

    // 5. Move to brick lifted (blocking call)
    if(!getStatus())
        return false;
    speed = getSpeed(SPEED);
    if(moveRobotWait(qBrickLifted, speed) == false)
        return false;

    // 6. Go to idle Q (when camera is taking pictures)
    /*if(!getStatus())
        return false;
    speed = getSpeed(SPEED);
    if(moveRobotWait(_idleQ, speed) == false)
        return false;*/

    // Log
    printConsole("Moving robot to release area!");

    // 7. Go to release-lego-to-mr Q
    if(!getStatus())
        return false;
    speed = getSpeed(SPEED);
    if(moveRobotWait(_releaseBrickQ, speed) == false)
        return false;

    // Log
    printConsole("Releasing brick!");

    // 8. Open gripper
    if(!getStatus())
        return false;
    if(PG70SetConf(qGripperOpen) == false)
        return false;
    //sleep(2);

    // Log
    printConsole("Moving robot to idle position!");

    // 9. Go back to idle Q
    if(!getStatus())
        return false;
    speed = getSpeed(SPEED);
    if(moveRobotWait(_idleQ, speed) == false)
        return false;

    res.success = true;
    return true;
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RC_Grasp");
    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");

    // Topic names
    std::string kukaService, PG70Service, scenePath, hmiConsolePub, hmiStatusSub;
    pNh.param<std::string>("KukaCmdServiceName", kukaService, "/KukaNode");
    pNh.param<std::string>("PG70CmdServiceName", PG70Service, "/PG70/PG70");
    pNh.param<std::string>("scenePath", scenePath, "/home/yonas/catkin_ws/src/rc_rsd/RC_KukaScene/Scene.wc.xml");
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");
    pNh.param<std::string>("hmi_status_sub", hmiStatusSub, "/rcHMI/status");
    pNh.param<double>("idleHeight", _idleQHeight, 0.3);
    pNh.param<double>("graspOffset", _graspOffset, 0.0025);
    pNh.param<double>("graspLifted", _graspLifted, 0.03);
    pNh.param<double>("xMax", _xMax, 0.15);
    pNh.param<double>("yMax", _yMax, 0.08);
    pNh.param<double>("gripperOpenoffset", _gripperOpenOffset, 0.015);

    // Create service calls
    _serviceKukaSetConf = nh.serviceClient<kuka_rsi::setConfiguration>(kukaService + "/SetConfiguration");
    _serviceKukaStop = nh.serviceClient<kuka_rsi::stopRobot>(kukaService + "/StopRobot");
    _serviceKukaGetConf = nh.serviceClient<kuka_rsi::getConfiguration>(kukaService + "/GetConfiguration");
    _serviceKukaGetIsMoving = nh.serviceClient<kuka_rsi::getIsMoving>(kukaService + "/IsMoving");
    _serviceKukaGetQueueSize = nh.serviceClient<kuka_rsi::getQueueSize>(kukaService + "/GetQueueSize");
    _servicePG70Move = nh.serviceClient<pg70::Move>(PG70Service + "/move");
    _servicePG70Stop = nh.serviceClient<pg70::Stop>(PG70Service + "/stop");
    _servicePG70Open = nh.serviceClient<pg70::Open>(PG70Service + "/open");
    ros::ServiceServer serviceGrabBrick = nh.advertiseService("/rcGrasp/grabBrick", grabBrickCallback);

    // Subscribe
    ros::Subscriber hmiStatusSubs = nh.subscribe(hmiStatusSub, 10, hmiStatusCallback);

    // Create publisher
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);

    // Setup RobWork
    // Load robwork workcell and device
    _workcell = rw::loaders::WorkCellLoader::Factory::load(scenePath);
    _device = _workcell->getDevices()[0];
    _state = _workcell->getDefaultState();

    // Check if loaded
    if(_device == NULL)
    {
        // Log
        printConsole("Device not found!");
        return -1;
    }

    // Find Brick Frame
    _brickFrame = _workcell->findFrame("Brick");
    if(!_brickFrame)
    {
        // Log
        printConsole("Cannot find Brick frame in Scene file!");
        return -1;
    }

    // Find Gripper Frame
    _gripperFrame = _workcell->findFrame("PG70.TCP");
    if(!_gripperFrame)
    {
        // Log
        printConsole("Cannot find PG70.TCP frame in Scene file!");
        return -1;
    }

    // Find Camera Frame
    _cameraFrame = _workcell->findFrame("Camera");
    if(!_cameraFrame)
    {
        // Log
        printConsole("Cannot find Camera frame in Scene file!");
        return -1;
    }

    // Set robot in default position
    const rw::math::Q qZero(6,0,0,0,0,0,0);
    _device->setQ(qZero, _state);

    // Get limits
    _limits = _device->getBounds();

    // Make new invkin object here
    _inverseKinGripper = new rw::invkin::JacobianIKSolver(_device, _gripperFrame, _state);
    _inverseKinGripper->setEnableInterpolation(true);
    _inverseKinGripper->setCheckJointLimits(true);

    _inverseKinCamera = new rw::invkin::JacobianIKSolver(_device, _cameraFrame, _state);
    _inverseKinCamera->setEnableInterpolation(true);
    _inverseKinCamera->setCheckJointLimits(true);

    // Load transformations
    _w2camera = rw::kinematics::Kinematics::worldTframe(_cameraFrame, _state);
    _w2brick = rw::kinematics::Kinematics::worldTframe(_brickFrame, _state);
    _w2base = _device->worldTbase(_state);

    // Calculate idle Q
    rw::math::Transform3D<> posOffset(rw::math::Vector3D<>(0, 0, _idleQHeight));
    rw::math::Transform3D<> w2brickoffset = _w2brick * posOffset;
    rw::math::Transform3D<> transform((inverse(_w2base)*w2brickoffset).P(), rw::math::RPY<>(M_PI, PITCH_OFFSET, 0));
    std::vector<rw::math::Q> qVec = _inverseKinCamera->solve(transform, _state);
    if(qVec.empty())
    {
        // Log
        printConsole("Idle Q: Error in inverse kinematics!");
        return -1;
    }
    _idleQ = qVec[0];

    // Calculate release brick Q
    rw::kinematics::Frame *mobileRobotFrame = _workcell->findFrame("MobileRobot");
    if(!mobileRobotFrame)
    {
        // Log
        printConsole("Cannot find MobileRobot frame in Scene file!");
        return -1;
    }
    rw::math::Transform3D<> w2mobilerobot = rw::kinematics::Kinematics::worldTframe(mobileRobotFrame, _state);
    transform = rw::math::Transform3D<>((inverse(_w2base)*w2mobilerobot).P(), rw::math::RPY<>(-M_PI/2.0, 0, M_PI));
    qVec = _inverseKinGripper->solve(transform, _state);
    if(qVec.empty())
    {
        // Log
        printConsole("Release brick: Error in inverse kinematics!");
        return -1;
    }
    _releaseBrickQ = qVec[0];

    // Initialize devices
    KukaSetConf(_idleQ);
    PG70Open();

    // Sleep rate
    ros::Rate r(10);

    // ROS Spin: Handle callbacks
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    // Return
    return 0;
}

void KukaStopRobot()
{
    if(CONNECT_KUKA)
    {
        // Stop robot
        kuka_rsi::stopRobot stopObj;
        if(!_serviceKukaStop.call(stopObj))
            printConsole("Failed to call the 'serviceKukaStopRobot'");
    }
}

bool KukaIsMoving()
{
    if(CONNECT_KUKA)
    {
        // Is moving
        kuka_rsi::getIsMoving isMoveObj;
        if(!_serviceKukaGetIsMoving.call(isMoveObj))
        {
            // Log
            printConsole("Failed to call the 'serviceKukaGetIsMoving'");
            return true;
        }
        else
            return isMoveObj.response.isMoving;
    }

    return false;
}

bool KukaSetConf(rw::math::Q q, rw::math::Q speed)
{
    if(CONNECT_KUKA)
    {
        if(q.size() != 6)
            return false;

        // Create setConfiguration service
        kuka_rsi::setConfiguration config;

        // Fill out information
        for(unsigned int i = 0; i<q.size(); i++)
        {
            config.request.q[i] = q(i);
            config.request.speed[i] = speed(i);
        }

        // Call service
        if(!_serviceKukaSetConf.call(config))
        {
            // Log
            printConsole("Failed to call the 'serviceKukaSetConf'");
            return false;
        }
        else
            return true;
    }

    return false;
}

int KukaGetQueueSize()
{
    if(CONNECT_KUKA)
    {
        // Get UR queue size (commands)
        kuka_rsi::getQueueSize SizeObj;
        if(!_serviceKukaGetQueueSize.call(SizeObj))
        {
            // Log
            printConsole("Failed to call the 'serviceKukaGetQueueSize'");
            return -1;
        }
        else
            return SizeObj.response.queueSize;
    }

    return -1;
}

rw::math::Q KukaGetConf()
{
    // Return object
    rw::math::Q qRet(6);

    if(CONNECT_KUKA)
    {
        // Get current position
        kuka_rsi::getConfiguration confObj;
        if(!_serviceKukaGetConf.call(confObj))
        {
            // Log
            printConsole("Failed to call the 'serviceKukaGetConf'");
            return qRet;
        }

        for(unsigned int i=0; i<qRet.size(); i++)
            qRet(i) = confObj.response.q[i];
    }

    return qRet;
}

bool PG70SetConf(rw::math::Q q)
{
    if(CONNECT_PG70)
    {
        if(q.size() != 1)
            return false;

        // Create setConfiguration service
        pg70::Move config;

        // Fill out information
        config.request.pos = q(0);

        // Call service
        if(!_servicePG70Move.call(config))
        {
            // Log
            printConsole("Failed to call the 'servicePG70Move'");
            return false;
        }
        else
            return true;
    }

    return true;
}

void PG70Stop()
{
    if(CONNECT_PG70)
    {
        // Log
        printConsole("Gripper stop!");

        // Stop
        pg70::Stop stopObj;
        if(!_servicePG70Stop.call(stopObj))
            printConsole("Failed to call the 'servicePG70Stop'");
    }
}

void PG70Open()
{
    if(CONNECT_PG70)
    {
        // Log
        printConsole("Gripper open!");

        // Open
        pg70::Open openObj;
        openObj.request.power = 10.0;
        if(!_servicePG70Open.call(openObj))
            printConsole("Failed to call the 'servicePG70Open'");
    }
}
