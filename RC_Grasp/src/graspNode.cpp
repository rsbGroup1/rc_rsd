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

#include <rc_grasp/setConfiguration.h>
#include <rc_grasp/getConfiguration.h>
#include <rc_grasp/stopRobot.h>
#include <rc_grasp/getIsMoving.h>
#include <rc_grasp/getQueueSize.h>
#include <rc_grasp/Move.h>
#include <rc_grasp/Stop.h>
#include <rc_grasp/Open.h>
#include <rc_grasp/grabBrick.h>

#include <iostream>

// Defines
#define DEGREETORAD                 (M_PI/180.0)
#define RADTODEGREE                 (180.0/M_PI)
#define ROBOT_NAME                  "KukaKR6R700"
#define CONNECT_KUKA                true
#define CONNECT_PG70                false
#define PITCH_OFFSET                (-17.340*DEGREETORAD)

// Global variables
ros::ServiceClient _serviceKukaSetConf, _serviceKukaStop, _serviceKukaGetConf, _serviceKukaGetQueueSize, _serviceKukaGetIsMoving;
ros::ServiceClient _servicePG70Move, _servicePG70Stop, _servicePG70Open;
rw::models::WorkCell::Ptr _workcell;
rw::models::Device::Ptr _device;
rw::kinematics::State _state;
rw::invkin::JacobianIKSolver *_inverseKinGripper, *_inverseKinCamera;
rw::kinematics::Frame *_gripperFrame, *_cameraFrame, *_brickFrame;
rw::math::Transform3D<> _w2base, _w2camera, _w2brick;
double _idleQHeight, _xMax, _yMax, _graspOffset, _graspLifted;
rw::math::Q _idleQ, _releaseBrickQ;
rw::models::Device::QBox _limits;
ros::Publisher _hmiConsolePub;

// Prototypes
void KukaStopRobot();
bool KukaIsMoving();
bool KukaSetConf(rw::math::Q q);
int KukaGetQueueSize();
rw::math::Q KukaGetConf();
bool PG70SetConf(rw::math::Q q);
void PG70Stop();
void PG70Open();

// Functions
bool moveRobotWait(rw::math::Q q)
{
    if(KukaSetConf(q))
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
    ROS_ERROR_STREAM(msg.c_str());
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

    if(p[0]>_xMax || p[0]<-_xMax || p[1]>_yMax || p[1]<-_yMax)
    {
        printConsole("Position larger than workspace!");
        return qRet;
    }

    // Inverse kin
    rw::math::Transform3D<> posOffset(p);
    rw::math::Transform3D<> w2brickOffset = _w2brick * posOffset;
    rw::math::Transform3D<> transform((inverse(_w2base)*w2brickOffset).P(), rw::math::RPY<>(M_PI, PITCH_OFFSET, M_PI));
    std::cout << transform << std::endl;
    std::vector<rw::math::Q> qVec = _inverseKinGripper->solve(transform, _state);
    if(qVec.empty())
    {
        printConsole("Pick up frame: Error in inverse kinematics!");
        return qRet;
    }

    qRet = qVec[0];

    // Fix rotation
    if(qRet[5] + rotation < _limits.first[5])
        printConsole("Joint 5 reached its limit!");
    else if(qRet[5] + rotation > _limits.second[5])
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

bool grabBrickCallback(rc_grasp::grabBrick::Request &req, rc_grasp::grabBrick::Response &res)
{
    /*req.x;
    req.y;
    req.theta;
    req.size;*/

    // Default return message
    res.success = false;

    // Configuration for lego brick
    rw::math::Q qBrickLifted = getQFromPinBrickFrame(rw::math::Vector3D<>(req.x, req.y, _graspLifted), req.theta);
    rw::math::Q qBrick = getQFromPinBrickFrame(rw::math::Vector3D<>(req.x, req.y, _graspOffset), req.theta);

    if(checkQ(qBrickLifted) == false || checkQ(qBrick) == false)
        return false;

    // 1. Move to brick lifted (blocking call)
    if(moveRobotWait(qBrickLifted) == false)
        return false;

    // 2. Open gripper
    // Gripper max cmd: 0.034m. Equals an opening of 0.068m. If 5cm opening is wanted: Send 0.025
    rw::math::Q qGripperOpen(1, req.size);
    if(PG70SetConf(qGripperOpen) == false)
        return false;

    // 3. Move to brick down (blocking call)
    if(moveRobotWait(qBrick) == false)
        return false;

    // 4. Close gripper to req.size
    rw::math::Q qGripperClose(1, req.size/2.0);
    if(PG70SetConf(qGripperClose) == false)
        return false;

    // 5. Go to idle Q (when camera is taking pictures)
    if(moveRobotWait(_idleQ) == false)
        return false;

    // 6. Go to release-lego-to-mr Q
    if(moveRobotWait(_releaseBrickQ) == false)
        return false;

    // 7. Open gripper
    PG70Open();

    // 8. Go back to idle Q
    if(moveRobotWait(_idleQ) == false)
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
    ros::init(argc, argv, "RSD_Grasp_Node");
    ros::NodeHandle nh, pNh("~");

    // Topic names
    std::string kukaService, PG70Service, grabBrickService, scenePath, hmiConsolePub;
    pNh.param<std::string>("KukaCmdServiceName", kukaService, "/KukaNode");
    pNh.param<std::string>("PG70CmdServiceName", PG70Service, "/PG70/PG70");
    pNh.param<std::string>("grabBrickServiceName", grabBrickService, "/mrGrasp/grabBrick");
    pNh.param<std::string>("scenePath", scenePath, "/home/student/catkin_ws/src/rc_rsd/RC_KukaScene/Scene.wc.xml");
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/mrHMI/console");
    pNh.param<double>("idleHeight", _idleQHeight, 0.3);
    pNh.param<double>("graspOffset", _graspOffset, 0.0);
    pNh.param<double>("graspLifted", _graspLifted, 0.1);
    pNh.param<double>("xMax", _xMax, 0.15);
    pNh.param<double>("yMax", _yMax, 0.08);

    // Create service calls
    _serviceKukaSetConf = nh.serviceClient<rc_grasp::setConfiguration>(kukaService + "/SetConfiguration");
    _serviceKukaStop = nh.serviceClient<rc_grasp::stopRobot>(kukaService + "/StopRobot");
    _serviceKukaGetConf = nh.serviceClient<rc_grasp::getConfiguration>(kukaService + "/GetConfiguration");
    _serviceKukaGetIsMoving = nh.serviceClient<rc_grasp::getIsMoving>(kukaService + "/IsMoving");
    _serviceKukaGetQueueSize = nh.serviceClient<rc_grasp::getQueueSize>(kukaService + "/GetQueueSize");
    _servicePG70Move = nh.serviceClient<rc_grasp::Move>(PG70Service + "/move");
    _servicePG70Stop = nh.serviceClient<rc_grasp::Stop>(PG70Service + "/stop");
    _servicePG70Open = nh.serviceClient<rc_grasp::Open>(PG70Service + "/open");
    ros::ServiceServer serviceGrabBrick = nh.advertiseService(grabBrickService, grabBrickCallback);

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
        printConsole("Device not found!");
        return -1;
    }

    // Find Brick Frame
    _brickFrame = _workcell->findFrame("Brick");
    if(!_brickFrame)
    {
        printConsole("Cannot find Brick frame in Scene file!");
        return -1;
    }

    // Find Gripper Frame
    _gripperFrame = _workcell->findFrame("PG70.TCP");
    if(!_gripperFrame)
    {
        printConsole("Cannot find PG70.TCP frame in Scene file!");
        return -1;
    }

    // Find Camera Frame
    _cameraFrame = _workcell->findFrame("Camera");
    if(!_cameraFrame)
    {
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
        printConsole("Idle Q: Error in inverse kinematics!");
        return -1;
    }
    _idleQ = qVec[0];

    // Calculate release brick Q
    rw::kinematics::Frame *mobileRobotFrame = _workcell->findFrame("MobileRobot");
    if(!mobileRobotFrame)
    {
        printConsole("Cannot find MobileRobot frame in Scene file!");
        return -1;
    }
    rw::math::Transform3D<> w2mobilerobot = rw::kinematics::Kinematics::worldTframe(mobileRobotFrame, _state);
    transform = rw::math::Transform3D<>((inverse(_w2base)*w2mobilerobot).P(), rw::math::RPY<>(-M_PI/2.0, 0, M_PI));
    qVec = _inverseKinGripper->solve(transform, _state);
    if(qVec.empty())
    {
        printConsole("Release brick: Error in inverse kinematics!");
        return -1;
    }
    _releaseBrickQ = qVec[0];

    // Initialize devices
    KukaSetConf(_idleQ);
    PG70Open();

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

void KukaStopRobot()
{
    if(CONNECT_KUKA)
    {
        // Stop robot
        rc_grasp::stopRobot stopObj;
        if(!_serviceKukaStop.call(stopObj))
            printConsole("Failed to call the 'serviceKukaStopRobot'");
    }
}

bool KukaIsMoving()
{
    if(CONNECT_KUKA)
    {
        // Is moving
        rc_grasp::getIsMoving isMoveObj;
        if(!_serviceKukaGetIsMoving.call(isMoveObj))
        {
            printConsole("Failed to call the 'serviceKukaGetIsMoving'");
            return true;
        }
        else
            return isMoveObj.response.isMoving;
    }

    return false;
}

bool KukaSetConf(rw::math::Q q)
{
    if(CONNECT_KUKA)
    {
        if(q.size() != 6)
            return false;

        // Create setConfiguration service
        rc_grasp::setConfiguration config;

        // Fill out information
        for(unsigned int i = 0; i<q.size(); i++)
            config.request.q[i] = q(i);

        // Call service
        if(!_serviceKukaSetConf.call(config))
        {
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
        rc_grasp::getQueueSize SizeObj;
        if(!_serviceKukaGetQueueSize.call(SizeObj))
        {
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
        rc_grasp::getConfiguration confObj;
        if(!_serviceKukaGetConf.call(confObj))
        {
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
        rc_grasp::Move config;

        // Fill out information
        config.request.pos = q(0);

        // Call service
        if(!_servicePG70Move.call(config))
        {
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
        // Stop
        rc_grasp::Stop stopObj;
        if(!_servicePG70Stop.call(stopObj))
            printConsole("Failed to call the 'servicePG70Stop'");
    }
}

void PG70Open()
{
    if(CONNECT_PG70)
    {
        // Open
        rc_grasp::Open openObj;
        openObj.request.power = 10.0;
        if(!_servicePG70Open.call(openObj))
            printConsole("Failed to call the 'servicePG70Open'");
    }
}
