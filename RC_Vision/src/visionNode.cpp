// Includes
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <rc_vision/getBricks.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv/cv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <boost/thread/mutex.hpp>

// Global variables
cv::Mat _image;
boost::mutex _mImage;
ros::Publisher _anyBricksPub, _hmiConsolePub;

// Functions
void printConsole(std::string msg)
{
    ROS_ERROR_STREAM(msg.c_str());
    std_msgs::String pubMsg;
    pubMsg.data = "Vision: " + msg;
    _hmiConsolePub.publish(pubMsg);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Acquire lock on the image
    boost::unique_lock<boost::mutex> lock(_mImage);

    // Convert to OpenCV
    _image = cv_bridge::toCvShare(msg, "bgr8")->image;
}

bool analyzeFrameCallback(rc_vision::getBricks::Request &req, rc_vision::getBricks::Response &res)
{
    // TODO

    return true;
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "rc_vision");
    ros::NodeHandle nh;
    ros::NodeHandle pNh(ros::this_node::getName() + "/");

    // Topic names
    std::string imageSub, anyBricksPub, hmiConsolePub, analyzeService;
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");
    pNh.param<std::string>("image_sub", imageSub, "/rcCamera/image_raw");
    pNh.param<std::string>("any_brick_pub", anyBricksPub, "/rcVision/anyBricks");
    pNh.param<std::string>("getBricks_service", analyzeService, "/rcVision/getBricks");
    ros::ServiceServer analyzeFrameService = nh.advertiseService(analyzeService, analyzeFrameCallback);

    // Subscribers
    image_transport::ImageTransport itImg(nh);
    image_transport::Subscriber subImg = itImg.subscribe(imageSub, 1, imageCallback);

    // Publishers
    _anyBricksPub = nh.advertise<std_msgs::Bool>(anyBricksPub, 1);
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);

    // Set loop rate
    ros::Rate loop_rate(10);

    while(nh.ok())
    {
        std::cout << "OK" << std::endl;

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Return
    return 0;
}
