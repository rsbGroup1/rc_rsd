// Includes
#include <ros/ros.h>
#include "std_msgs/Bool.h"
//#include "vision"
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
ros::Publisher _anyBricksPublisher;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Acquire lock on the image
    boost::unique_lock<boost::mutex> lock(_mImage);

    // Convert to OpenCV
    _image = cv_bridge::toCvShare(msg, "bgr8")->image;
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RSD_Vision_Node");
    ros::NodeHandle nh, pNh("~");

    // Topic names
    std::string imageSub, anyBricksPub;
    pNh.param<std::string>("image_sub", imageSub, "/rcCamera/image_raw");
    pNh.param<std::string>("any_brick_pub", anyBricksPub, "/rcVision/anyBricks");

    // Subscribers
    image_transport::ImageTransport itImg(nh);
    image_transport::Subscriber subImg = itImg.subscribe(imageSub, 1, imageCallback);

    // Publishers
    _anyBricksPublisher = nh.advertise<std_msgs::Bool>(anyBricksPub, 1);

    // Set loop rate
    ros::Rate loop_rate(1);

    while(nh.ok())
    {
        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Return
    return 0;
}
