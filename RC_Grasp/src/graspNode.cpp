// Includes
#include <ros/ros.h>
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
#include <iostream>

// Defines
#define ROBOT_NAME                  "UR1"

// Global variables
ros::Publisher _safetyPublisher;

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RSD_Grasp_Node");
    ros::NodeHandle nh;

    // Topic names
    std::string safetyPub;
    nh.param<std::string>("/RC_Grasp/Grasp/grasp_pub", safetyPub, "/rcSafety/safetyStatus");

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
