// Includes
#include <ros/ros.h>
#include <iostream>

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RSD_MESServer_Node");
    ros::NodeHandle nh, pNh("~");

    // Topic names
    std::string imagePub;
    pNh.param<std::string>("image_pub", imagePub, "/rcCamera/image");

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
