// Includes
#include <ros/ros.h>
#include <iostream>

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
