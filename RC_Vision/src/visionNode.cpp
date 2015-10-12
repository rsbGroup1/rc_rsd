// Includes
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <rc_vision/getBricks.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv/cv.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <queue>

// Queue class
template <typename T>
class SynchronisedQueue
{
    private:
        std::queue<T> m_queue;              // Use STL queue to store data
        boost::mutex m_mutex;               // The mutex to synchronise on
        boost::condition_variable m_cond;   // The condition to wait for

    public:
        // Add data to the queue and notify others
        void enqueue(const T& data)
        {
            // Acquire lock on the queue
            boost::unique_lock<boost::mutex> lock(m_mutex);

            // Add the data to the queue
            m_queue.push(data);

            // Notify others that data is ready
            m_cond.notify_one();
        }

        // Get data from the queue. Wait for data if not available
        T dequeue()
        {
            // Acquire lock on the queue
            boost::unique_lock<boost::mutex> lock(m_mutex);

            // When there is no data, wait till someone fills it.
            // Lock is automatically released in the wait and obtained
            // again after the wait
            while(m_queue.size()==0)
                m_cond.wait(lock);

            // Retrieve the data from the queue
            T result = m_queue.front();
            m_queue.pop();

            return result;
        }

        int size()
        {
            // Acquire lock on the queue
            boost::unique_lock<boost::mutex> lock(m_mutex);
            return m_queue.size();
        }

        T back()
        {
            // Acquire lock on the queue
            boost::unique_lock<boost::mutex> lock(m_mutex);
            return m_queue.back();
        }

        void clear()
        {
            // Acquire lock on the queue
            boost::unique_lock<boost::mutex> lock(m_mutex);
            std::queue<T> empty;
            std::swap(m_queue, empty);
        }
};

// Global variables
SynchronisedQueue<cv::Mat> _queueImage;
ros::Publisher _anyBricksPub, _hmiConsolePub;
int _sMin = 145, _sMax = 255;
int _hMin = 0, _hMax = 255;
int _minBlobSize = 500;
int _closeKernelSize = 7;
int _colorUpperThres = 200;

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
    // Convert to OpenCV
    _queueImage.enqueue(cv_bridge::toCvShare(msg, "bgr8")->image);
}

bool analyzeFrameCallback(rc_vision::getBricks::Request &req, rc_vision::getBricks::Response &res)
{
    // TODO

    return true;
}

void printImageType(int number)
{
    // Find type
    int imgTypeInt = number%8;
    std::string imgTypeString;

    switch (imgTypeInt)
    {
        case 0:
            imgTypeString = "8U";
            break;
        case 1:
            imgTypeString = "8S";
            break;
        case 2:
            imgTypeString = "16U";
            break;
        case 3:
            imgTypeString = "16S";
            break;
        case 4:
            imgTypeString = "32S";
            break;
        case 5:
            imgTypeString = "32F";
            break;
        case 6:
            imgTypeString = "64F";
            break;
        default:
            break;
    }

    // Find channel
    int channel = (number/8) + 1;

    // Print
    std::cout << "CV_" << imgTypeString << "C" << channel << std::endl;
}

std::string determineColor(cv::Point3d point)
{
    bool red = (point.z>_colorUpperThres)?(true):(false);
    bool green = (point.y>_colorUpperThres)?(true):(false);
    bool blue = (point.x>_colorUpperThres)?(true):(false);

    if(red == true && green == false && blue == false)
        return "red";
    else if(red == false && green == true && blue == false)
        return "green";
    else if(red == false && green == false && blue == true)
        return "blue";
    else if(red == true && green == true && blue == false)
        return "yellow";
    else if(red == false && green == false && blue == false)
        return "black";
    else if(red == true && green == true && blue == true)
        return "white";
    else
        return "undefined";
}

std::vector<std::vector<cv::Point2i> > findBlobs(const cv::Mat &binary)
{
    std::vector<std::vector<cv::Point2i> > blobs;

    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    cv::Mat label_image;
    binary.convertTo(label_image, CV_32SC1);
    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y<label_image.rows; y++)
    {
        int *row = (int*)label_image.ptr(y);
        for(int x=0; x < label_image.cols; x++)
        {
            if(row[x] != 1)
                continue;

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 4);

            std::vector <cv::Point2i> blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++)
            {
                int *row2 = (int*)label_image.ptr(i);
                for(int j=rect.x; j < (rect.x+rect.width); j++)
                {
                    if(row2[j] != label_count)
                        continue;

                    blob.push_back(cv::Point2i(j,i));
                }
            }

            blobs.push_back(blob);
            label_count++;
        }
    }

    return blobs;
}

std::vector<std::vector<cv::Point2i> > filterAndFindBlobs(const cv::Mat &image)
{
    // Convert to HSV
    cv::Mat hsv, binary;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // Filter in HSV space
    cv::inRange(hsv, cv::Scalar(_hMin, _sMin, 0), cv::Scalar(_hMax, _sMax, 255), hsv);

    // Close image (erode -> dilate)
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2*_closeKernelSize + 1, 2*_closeKernelSize+1), cv::Point(_closeKernelSize, _closeKernelSize));
    cv::morphologyEx(hsv, hsv, cv::MORPH_CLOSE, element);

    // Threshold: Sets pixels to either 0 or 1 (needed by blob detection algorithm)
    cv::threshold(hsv, binary, 128, 1, 0);

    //cv::namedWindow("HSV", CV_WINDOW_KEEPRATIO);
    //cv::imshow("HSV", hsv);

    // Blob detection
    std::vector<std::vector<cv::Point2i> > blobs = findBlobs(binary), blobsFiltered;

    // Filter blobs by pixel size
    for(unsigned int i=0; i<blobs.size(); i++)
        if(blobs[i].size() > _minBlobSize)
            blobsFiltered.push_back(blobs[i]);

    return blobsFiltered;
}

void detectBricksThread()
{
    while(true)
    {
        try
        {
            cv::Mat image = _queueImage.dequeue();
            if(filterAndFindBlobs(image).size() > 0)
            {
                std_msgs::Bool msg;
                msg.data = true;
                _anyBricksPub.publish(msg);
            }

            // Signal interrupt point
            boost::this_thread::interruption_point();
        }
        catch(const boost::thread_interrupted&)
        {
            break;
        }
    }
}

void showBlobs(std::vector<std::vector<cv::Point2i> > blobs, cv::Size2i imageSize)
{
    // Show all blobs
    cv::Mat blobImg = cv::Mat::zeros(imageSize, CV_8UC3);
    for(unsigned int i=0; i<blobs.size(); i++)
    {
        unsigned char r = 255 * (rand()/(1.0 + RAND_MAX));
        unsigned char g = 255 * (rand()/(1.0 + RAND_MAX));
        unsigned char b = 255 * (rand()/(1.0 + RAND_MAX));

        for(unsigned int  j=0; j<blobs[i].size(); j++)
        {
            int x = blobs[i][j].x;
            int y = blobs[i][j].y;

            blobImg.at<cv::Vec3b>(y,x)[0] = b;
            blobImg.at<cv::Vec3b>(y,x)[1] = g;
            blobImg.at<cv::Vec3b>(y,x)[2] = r;
        }
    }
    cv::namedWindow("Blobs", CV_WINDOW_KEEPRATIO);
    cv::imshow("Blobs", blobImg);
}

double lengthOfPoint(cv::Point2f point)
{
    return sqrt(point.x*point.x + point.y*point.y);
}

void findBricks(const cv::Mat &image)
{
    // Get blobs
    std::vector<std::vector<cv::Point2i> > blobs = filterAndFindBlobs(image);

    // Show blobs
    //showBlobs(blobs, image.size());

    // Determine color by getting average pixel value of blobs
    std::vector<cv::Point3d> points;
    points.resize(blobs.size());
    for(unsigned int i=0; i<blobs.size(); i++)
    {
        for(unsigned int j=0; j<blobs[i].size(); j++)
        {
            int x = blobs[i][j].x;
            int y = blobs[i][j].y;
            double blobSize = blobs[i].size();
            points[i].x += image.at<cv::Vec3b>(y,x)[0] / blobSize;
            points[i].y += image.at<cv::Vec3b>(y,x)[1] / blobSize;
            points[i].z += image.at<cv::Vec3b>(y,x)[2] / blobSize;
        }
    }

    // Make OBB's
    cv::Mat img;
    image.copyTo(img);
    for(int k=0; k<blobs.size(); k++)
    {
        cv::RotatedRect box = cv::minAreaRect(cv::Mat(blobs[k]));
        cv::Point2f vertices[4];
        box.points(vertices);

        // Determine correct angle
        cv::Point2f longestLine;
        if(lengthOfPoint(vertices[0]-vertices[1]) > lengthOfPoint(vertices[1]-vertices[2]))
        {
            longestLine = vertices[0]-vertices[1];
            cv::line(img, vertices[0], vertices[1], cv::Scalar(255, 0, 0), 5);
        }
        else
        {
            longestLine = vertices[1]-vertices[2];
            cv::line(img, vertices[1], vertices[2], cv::Scalar(255, 0, 0), 5);
        }

        double angle = atan2(longestLine.y, longestLine.x) * (180.0/M_PI) * -1;
        if(angle < -90.0)
            angle += 180.0;

        std::cout << "Box " << k << ": Angle: " << angle << " Center: " << box.center << " Size: " << box.size << " Color: " << determineColor(points[k]) << std::endl;

        for(int i=0; i<4; ++i)
            cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 0, 0), 1, CV_AA);
    }
    cv::namedWindow("OBB", CV_WINDOW_KEEPRATIO);
    cv::imshow("OBB", img);

    cv::waitKey(0);
}

int main()
{
    // Setup ROS Arguments
    /*char** argv = NULL;
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

    // Brick detection thread
    boost::thread detectionThread(detectBricksThread);

    // Set loop rate
    ros::Rate loop_rate(10);

    while(nh.ok())
    {
        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }*/

    // Load image
    cv::Mat image = cv::imread("/home/yonas/Desktop/Lego/1.jpg", CV_LOAD_IMAGE_UNCHANGED);

    // Check if picture is loaded successfully
    if(!image.data)
    {
       printf("No image \n");
       return 0;
    }

    // Test call
    findBricks(image);

    // Interrupt thread
    //detectionThread.interrupt();

    // Return
    return 0;
}
