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
int _hMin = 0, _hMax = 255;
int _sMin = 80, _sMax = 255;
int _vMin = 50, _vMax = 255;
int _minBlobSize = 500; // 500
int _minLegoArea = 0, _maxLegoArea = 50000; // 5000, 20000
int _closeKernelSize = 5;
int _pixelToM = 3200; //3070;   // Pixels per meter
int _baseLegoSize = 50; // Half the width of all bricks: 1 tap on lego brick
double _xMax, _yMax;
cv::Size2i _imageSize(1280,720);
boost::mutex _blobMutex, _paramMutex;
std::vector<BlobColor> _blobs;
cv::Mat _blobImage;
int xPoint = ((double)_imageSize.width/_pixelToM-0.3)*_pixelToM/2.0;
cv::Point2f _tl(0, 90);
cv::Point2f _br(_imageSize.width-0, 590);

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
    std::string retString;
    double sum = point.x+point.y+point.z;
    double redAmount = point.z/sum;
    double greenAmount = point.y/sum;
    double blueAmount = point.x/sum;

    //std::cout << redAmount << " " << greenAmount << " " << blueAmount << " ";

    if(redAmount > 0.3 && greenAmount < 0.3 && blueAmount < 0.3)
        retString = "red";
    else if(redAmount < 0.3 && greenAmount > 0.3 && blueAmount < 0.3)
        retString = "green";
    else if(redAmount < 0.5 && greenAmount < 0.5 && blueAmount > 0.3)
        retString = "blue";
    else if(redAmount > 0.3 && greenAmount > 0.3 && blueAmount < 0.3)
        retString = "yellow";
    else
        retString = "undefined";

    //std::cout << retString << std::endl;

    return retString;
}

std::vector<BlobColor> findBlobs(const cv::Mat &binary)
{
    std::vector<BlobColor> blobs;

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

            BlobColor blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++)
            {
                int *row2 = (int*)label_image.ptr(i);
                for(int j=rect.x; j < (rect.x+rect.width); j++)
                {
                    if(row2[j] != label_count)
                        continue;

                    blob.blob.push_back(cv::Point2i(j,i));
                }
            }

            blobs.push_back(blob);
            label_count++;
        }
    }

    return blobs;
}

std::vector<BlobColor> filterAndFindBlobs(const cv::Mat &image)
{
    _paramMutex.lock();
    int hMax = _hMax, hMin = _hMin;
    int sMax = _sMax, sMin = _sMin;
    int vMax = _vMax, vMin = _vMin;
    int closeKernelSize = _closeKernelSize;
    int minBlobSize = _minBlobSize;
    _paramMutex.unlock();

    // Return vector
    std::vector<BlobColor> blobColorVec, blobColorFilteredVec;

    // Convert to HSV
    cv::Mat hsv, binary;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // Filter in HSV space
    cv::inRange(hsv, cv::Scalar(hMin, sMin, vMin), cv::Scalar(hMax, sMax, vMax), hsv);

    // Close image (erode -> dilate)
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2*closeKernelSize + 1, 2*closeKernelSize+1), cv::Point(closeKernelSize, closeKernelSize));
    cv::morphologyEx(hsv, hsv, cv::MORPH_CLOSE, element);

    // Threshold: Sets pixels to either 0 or 1 (needed by blob detection algorithm)
    cv::threshold(hsv, binary, 128, 1, 0);

    // Blob detection
    blobColorVec = findBlobs(binary);

    // Filter blobs by number of pixels
    for(unsigned int i=0; i<blobColorVec.size(); i++)
        if(blobColorVec[i].blob.size() > minBlobSize)
            blobColorFilteredVec.push_back(blobColorVec[i]);

    // Filter blobs by color
    // Determine color by getting average pixel value of blobs
    std::vector<cv::Point3d> points;
    points.resize(blobColorFilteredVec.size());
    for(unsigned int i=0; i<blobColorFilteredVec.size(); i++)
    {
        for(unsigned int j=0; j<blobColorFilteredVec[i].blob.size(); j++)
        {
            int x = blobColorFilteredVec[i].blob[j].x;
            int y = blobColorFilteredVec[i].blob[j].y;
            double blobSize = blobColorFilteredVec[i].blob.size();
            points[i].x += image.at<cv::Vec3b>(y,x)[0] / blobSize;
            points[i].y += image.at<cv::Vec3b>(y,x)[1] / blobSize;
            points[i].z += image.at<cv::Vec3b>(y,x)[2] / blobSize;
        }

        // Determine color and add
        std::string color = determineColor(points[i]);
        blobColorFilteredVec[i].color = color;
    }

    // Return
    return blobColorFilteredVec;
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
            //cv::imwrite("/home/yonas/Desktop/image.jpg", image);

            // Test call
            cv::Mat croppedImage = image(cv::Rect(_tl.x, _tl.y, _br.x-_tl.x, _br.y-_tl.y));

            // Detect blobs
            std::vector<BlobColor> blob = filterAndFindBlobs(croppedImage);

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

cv::Point2f convertPixelToM(cv::Point2f pointPixel, const cv::Mat &image)
{
    // Return val
    cv::Point2f pointM;

    // Offset pixels
    pointPixel += _tl;
    pointPixel.x -= image.cols/2.0;
    pointPixel.y -= image.rows/2.0;
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
                brick.posX = convertPixelToM(box.center, image).x;
                brick.posY = convertPixelToM(box.center, image).y;
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

                // Add to vector if in robot reach
                int lineThickness = 5;
                if(-_xMax < brick.posX && brick.posX < _xMax && -_yMax < brick.posY && brick.posY < _yMax )
                    brickVec.push_back(brick);
                else
                    lineThickness = 2;

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
                    cv::line(img, vertices[i]+_tl, vertices[(i + 1) % 4]+_tl, color, lineThickness, CV_AA);

                // Draw center in image
                box.center += _tl;
                cv::circle(img, box.center, 3, cv::Scalar(255,255,255), 5);
            }
        }

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
    else if(func == "vmin")
        _vMin = value;
    else if(func == "vmax")
        _vMax = value;
    else if(func == "area")
        _minLegoArea = value;
    else if(func == "blob")
        _minBlobSize = value;
    else if(func == "close")
        _closeKernelSize = value;
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
    std::string imageSub, anyBricksPub, hmiConsolePub, analyzeService, visionParamSub, imagePub;
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");
    pNh.param<std::string>("image_sub", imageSub, "/RC_Camera/image_rect_color");
    pNh.param<std::string>("any_brick_pub", anyBricksPub, "/rcVision/anyBricks");
    pNh.param<std::string>("getBricks_service", analyzeService, "/rcVision/getBricks");
    pNh.param<std::string>("visionParamSub", visionParamSub, "/rcHMI/visionParam");
    pNh.param<std::string>("visionImagePub", imagePub, "/rcVision/image");
    pNh.param<double>("xMax", _xMax, 0.15);
    pNh.param<double>("yMax", _yMax, 0.08);

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

    // Loop
    while(ros::ok())
        ros::spinOnce();

    // Interrupt thread
    detectionThread.interrupt();

    // Return
    return 0;
}
