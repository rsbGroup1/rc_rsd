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
#include <string>

// Defines
#define DEGREETORAD             (M_PI/180.0)
#define RADTODEGREE             (180.0/M_PI)

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

// Brick struct
struct Brick
{
    std::string color;
    float size;
    float posX;
    float posY;
    float theta;
};

struct BlobColor
{
    std::vector<cv::Point2i> blob;
    std::string color;
};

// Global variables
SynchronisedQueue<cv::Mat> _queueImage;
ros::Publisher _anyBricksPub, _hmiConsolePub;
image_transport::Publisher _imagePub;
int _sMin = 150, _sMax = 255;
int _hMin = 0, _hMax = 255;
int _minBlobSize = 500;
int _minLegoArea = 5000, _maxLegoArea = 20000;
int _closeKernelSize = 9;
int _colorUpperThres = 100;
int _pixelToM = 3070;   // Pixels per meter
int _baseLegoSize = 50; // Half the width of all bricks: 1 tap on lego brick
cv::Size2i _imageSize(1280,720);
boost::mutex _blobMutex, _paramMutex;
std::vector<BlobColor> _blobs;
cv::Mat _blobImage;

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
    cv::waitKey(0);
}

std::vector<BlobColor> filterAndFindBlobs(const cv::Mat &image)
{
    _paramMutex.lock();
    int hMax = _hMax, hMin = _hMin;
    int sMax = _sMax, sMin = _sMin;
    int closeKernelSize = _closeKernelSize;
    int minBlobSize = _minBlobSize;
    _paramMutex.unlock();

    // Return vector
    std::vector<BlobColor> blobColorVec;

    // Convert to HSV
    cv::Mat hsv, binary;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // Filter in HSV space
    cv::inRange(hsv, cv::Scalar(hMin, sMin, 0), cv::Scalar(hMax, sMax, 255), hsv);

    // Close image (erode -> dilate)
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2*closeKernelSize + 1, 2*closeKernelSize+1), cv::Point(closeKernelSize, closeKernelSize));
    cv::morphologyEx(hsv, hsv, cv::MORPH_CLOSE, element);

    // Threshold: Sets pixels to either 0 or 1 (needed by blob detection algorithm)
    cv::threshold(hsv, binary, 128, 1, 0);

    // Blob detection
    std::vector<std::vector<cv::Point2i> > blobs = findBlobs(binary), blobsFiltered;

    // Filter blobs by pixel size
    for(unsigned int i=0; i<blobs.size(); i++)
        if(blobs[i].size() > minBlobSize)
            blobsFiltered.push_back(blobs[i]);

    // Filter blobs by color
    // Determine color by getting average pixel value of blobs
    std::vector<cv::Point3d> points;
    points.resize(blobsFiltered.size());
    for(unsigned int i=0; i<blobsFiltered.size(); i++)
    {
        for(unsigned int j=0; j<blobsFiltered[i].size(); j++)
        {
            int x = blobsFiltered[i][j].x;
            int y = blobsFiltered[i][j].y;
            double blobSize = blobsFiltered[i].size();
            points[i].x += image.at<cv::Vec3b>(y,x)[0] / blobSize;
            points[i].y += image.at<cv::Vec3b>(y,x)[1] / blobSize;
            points[i].z += image.at<cv::Vec3b>(y,x)[2] / blobSize;
        }
    }

    for(unsigned int i=0; i<blobsFiltered.size(); i++)
    {
        std::string color = determineColor(points[i]);
        if(color == "red" || color == "yellow" || color == "blue")
        {
            BlobColor blobColorElement;
            blobColorElement.blob = blobsFiltered[i];
            blobColorElement.color = color;
            blobColorVec.push_back(blobColorElement);
        }
    }

    // Show
    //cv::namedWindow("HSV", CV_WINDOW_KEEPRATIO);
    //cv::imshow("HSV", hsv);

    // Return
    return blobColorVec;
}

void setBlobs(std::vector<BlobColor>  blobColor)
{
    boost::unique_lock<boost::mutex> lock(_blobMutex);
    _blobs = blobColor;
}

std::vector<BlobColor> getBlobs()
{
    boost::unique_lock<boost::mutex> lock(_blobMutex);
    std::vector<BlobColor> blobRet = _blobs;
    return blobRet;
}

void detectBricksThread()
{
    while(true)
    {
        try
        {
            // Get image from queue
            cv::Mat image = _queueImage.dequeue();

            // Detect blobs
            std::vector<BlobColor> blob = filterAndFindBlobs(image);

            // Filter by color, size etc
            // TODO:

            // Update global blob
            setBlobs(blob);

            // Check if any blobs
            std_msgs::Bool msg;
            if(blob.size() > 0)
            {
                msg.data = true;
                boost::unique_lock<boost::mutex> lock(_blobMutex);
                image.copyTo(_blobImage);
            }
            else
            {
                msg.data = false;
            }
            _anyBricksPub.publish(msg);

            // Signal interrupt point
            boost::this_thread::interruption_point();
        }
        catch(const boost::thread_interrupted&)
        {
            break;
        }
    }
}

double lengthOfPoint(cv::Point2f point)
{
    return sqrt(point.x*point.x + point.y*point.y);
}

cv::Point2f convertPixelToM(cv::Point2f pointPixel)
{
    // Return val
    cv::Point2d pointM;

    // Offset pixels
    pointPixel.x -= _imageSize.width/2.0;
    pointPixel.y -= _imageSize.height/2.0;
    pointPixel.y *= -1.0;

    // Convert
    pointM.x = pointPixel.x / _pixelToM;
    pointM.y = pointPixel.y / _pixelToM;

    // Return
    return pointM;
}

std::vector<Brick> findBricks()
{
    _paramMutex.lock();
    int minLegoArea = _minLegoArea;
    _paramMutex.unlock();

    // Get blobs
    std::vector<BlobColor> blobColorVec = getBlobs();

    // Return var
    std::vector<Brick> brickVec;

    // If any blobs
    if(blobColorVec.size() > 0)
    {
        // Show blobs
        //showBlobs(blobs, image.size());

        // Get image
        _blobMutex.lock();
        cv::Mat image;
        _blobImage.copyTo(image);
        _blobMutex.unlock();

        // Make OBB's
        cv::Mat img;
        image.copyTo(img);
        for(int k=0; k<blobColorVec.size(); k++)
        {
            cv::RotatedRect box = cv::minAreaRect(cv::Mat(blobColorVec[k].blob));

            if(minLegoArea<box.boundingRect().area()  && box.boundingRect().area()<_maxLegoArea)
            {
                cv::Point2f vertices[4];
                box.points(vertices);

                // Determine correct angle
                cv::Point2f longestLine;
                if(lengthOfPoint(vertices[0]-vertices[1]) > lengthOfPoint(vertices[1]-vertices[2]))
                    longestLine = vertices[0]-vertices[1];
                else
                    longestLine = vertices[1]-vertices[2];

                // Fix angle
                double angle = atan2(longestLine.y, longestLine.x);
                if(angle> M_PI/2.0)
                    angle -= M_PI;
                angle = (angle<0)?(-1*(M_PI/2.0 + angle)):(M_PI/2.0-angle);

                // Create new brick
                Brick brick;
                brick.color = blobColorVec[k].color;
                brick.theta = angle;
                box.center = convertPixelToM(box.center);
                brick.posX = box.center.x;
                brick.posY = box.center.y;
                brick.size = (box.size.height>box.size.width?box.size.height:box.size.width);
                brick.size = round(brick.size/_baseLegoSize) * 2;

                // Choose shortest angle if brick is 2x2
                if(brick.size == 2)
                {
                    if(brick.theta >= M_PI/4.0)
                        brick.theta -= M_PI/2.0;
                    else if(brick.theta <= -M_PI/4.0)
                        brick.theta += M_PI/2.0;
                }

                // Add to vector
                brickVec.push_back(brick);

                // Print to screen
                //std::cout << "Box " << k << ": Area: " << box.boundingRect().area() <<  " Blob: " << blobColorVec[k].blob.size() << " Angle: " << brick.theta*RADTODEGREE << " Center: " << box.center << " Size: " << box.size << " Color: " << blobColorVec[k].color << " " << std::endl;

                // Show correspondant color on image
                cv::Scalar color;
                if(blobColorVec[k].color == "red")
                    color = cv::Scalar(0,0,255);
                else if(blobColorVec[k].color == "blue")
                    color = cv::Scalar(255,0,0);
                else if(blobColorVec[k].color== "green")
                    color = cv::Scalar(0,255,0);
                else if(blobColorVec[k].color == "yellow")
                    color = cv::Scalar(0,255,255);
                else if(blobColorVec[k].color == "white")
                    color = cv::Scalar(255,255,255);

                // Draw box in image
                for(int i=0; i<4; ++i)
                    cv::line(img, vertices[i], vertices[(i + 1) % 4], color, 5, CV_AA);
            }
        }

        // Show
        //cv::namedWindow("OBB", CV_WINDOW_KEEPRATIO);
        //cv::imshow("OBB", img);

        // Convert to ROS format
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

        // Publish to topic
        _imagePub.publish(msg);
    }

    return brickVec;
}

bool analyzeFrameCallback(rc_vision::getBricks::Request &req, rc_vision::getBricks::Response &res)
{
    // Fetch image and analyze
    std::vector<Brick> bricks = findBricks();

    // Resize return vectors
    res.color.resize(bricks.size());
    res.theta.resize(bricks.size());
    res.posX.resize(bricks.size());
    res.posY.resize(bricks.size());
    res.size.resize(bricks.size());

    // Copy info
    for(unsigned i=0; i<bricks.size(); i++)
    {
        res.color[i] = bricks[i].color;
        res.theta[i] = bricks[i].theta;
        res.posX[i] = bricks[i].posX;
        res.posY[i] = bricks[i].posY;
        res.size[i] = bricks[i].size;
    }

    // Return
    return true;
}

void paramCallback(std_msgs::String msg)
{
    boost::unique_lock<boost::mutex> lock(_paramMutex);

    std::string func, strValue;
    int index = msg.data.find("|");
    func = msg.data.substr(0, index);
    strValue = msg.data.substr(index+1, msg.data.size()-1);
    int value = atoi(strValue.c_str());

    if(func == "smin")
        _sMin = value;
    else if(func == "smax")
        _sMax = value;
    else if(func == "hmin")
        _hMin = value;
    else if(func == "hmax")
        _hMax = value;
    else if(func == "area")
        _minLegoArea = value;
    else if(func == "color")
        _colorUpperThres = value;
    else if(func == "blob")
        _minBlobSize = value;
    else if(func == "close")
        _closeKernelSize = value;
}

/*void trackbar(int, void*)
{
    filterAndFindBlobs(_image);
}*/

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
    std::string imageSub, anyBricksPub, hmiConsolePub, analyzeService, visionParamSub, imagePub;
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");
    pNh.param<std::string>("image_sub", imageSub, "/RC_Camera/image_rect_color");
    pNh.param<std::string>("any_brick_pub", anyBricksPub, "/rcVision/anyBricks");
    pNh.param<std::string>("getBricks_service", analyzeService, "/rcVision/getBricks");
    pNh.param<std::string>("visionParamSub", visionParamSub, "/rcHMI/visionParam");
    pNh.param<std::string>("visionImagePub", imagePub, "/rcVision/image");

    // Service
    ros::ServiceServer analyzeFrameService = nh.advertiseService(analyzeService, analyzeFrameCallback);

    // Subscribers
    image_transport::ImageTransport itImg(nh);
    image_transport::Subscriber subImg = itImg.subscribe(imageSub, 1, imageCallback);
    ros::Subscriber paramSub = nh.subscribe(visionParamSub, 10, paramCallback);

    // Publishers
    _anyBricksPub = nh.advertise<std_msgs::Bool>(anyBricksPub, 1);
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);
    _imagePub = itImg.advertise(imagePub, 1);

    // Brick detection thread
    boost::thread detectionThread(detectBricksThread);

    // Set loop rate
    ros::Rate loop_rate(10);

    while(nh.ok())
    {
        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Load image
    /*cv::Mat image = cv::imread("/home/yonas/Desktop/Lego/16.jpg", CV_LOAD_IMAGE_UNCHANGED);

    // Check if picture is loaded successfully
    if(!image.data)
    {
       printf("No image \n");
       return 0;
    }

    // Test call
    findBricks(image);
    cv::waitKey(0);

    /*image.copyTo(_image);
    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("Hue min: ", "Control", &_hMin, 255, trackbar);
    cv::createTrackbar("Hue max: ", "Control", &_hMax, 255, trackbar);
    cv::createTrackbar("Sat min: ", "Control", &_sMin, 255, trackbar);
    cv::createTrackbar("Sat max: ", "Control", &_sMax, 255, trackbar);
    trackbar(0, 0);
    cv::waitKey(0);*/

    // Interrupt thread
    detectionThread.interrupt();

    // Return
    return 0;
}
