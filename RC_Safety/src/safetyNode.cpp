// Includes
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <iostream>

// Global variables
ros::Publisher _safetyPublisher;

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RSD_Safety_Node");
    ros::NodeHandle nh;

    // Topic names
    std::string safetyPub;
    nh.param<std::string>("/RC_Safety/Safety/safety_pub", safetyPub, "/rcSafety/safetyStatus");

    // Create topic
    _safetyPublisher = nh.advertise<std_msgs::Bool>(safetyPub, 1);

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
