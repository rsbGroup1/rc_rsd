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
    cv::RotatedRect box;
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
int _sMin = 85, _sMax = 255;
int _vMin = 35, _vMax = 255;
int _minBlobSize = 500;
int _minRedSize = 3500, _maxRedSize = 6500;
int _minYellowSize = 5500, _maxYellowSize = 12000;
int _minBlueSize = 1200, _maxBlueSize = 3500;
int _closeKernelSize = 5;
int _pixelToM = 3200; //3070;   // Pixels per meter
int _baseLegoSize = 50; // Half the width of all bricks: 1 tap on lego brick
double _xMax, _yMax;
cv::Size2i _imageSize(1280,720);
boost::mutex _blobMutex, _paramMutex;
std::vector<BlobColor> _blobs;
cv::Mat _blobImage;
int xPoint = ((double)_imageSize.width/_pixelToM-0.3)*_pixelToM/2.0;
cv::Point2f _tl, _br;
double _graspWidthM = 0.0;
double _fingerWidthM = 0.0;

// Functions
void printConsole(std::string msg)
{
    //ROS_INFO_STREAM(msg.c_str());
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
    // Get amount of RGB
    std::string retString;
    double sum = point.x+point.y+point.z;
    double redAmount = point.z/sum;
    double greenAmount = point.y/sum;
    double blueAmount = point.x/sum;

    // Determine color
    if(redAmount > 0.3 && greenAmount < 0.3 && blueAmount < 0.3)
        retString = "red";
    /*else if(redAmount < 0.3 && greenAmount > 0.3 && blueAmount < 0.3)
        retString = "green";*/
    else if(redAmount > 0.23 && greenAmount > 0.28 && blueAmount > 0.35)
        retString = "tape";
    else if(redAmount < 0.5 && greenAmount < 0.5 && blueAmount > 0.3)
        retString = "blue";
    else if(redAmount > 0.3 && greenAmount > 0.3 && blueAmount < 0.3)
        retString = "yellow";
    else
        retString = "undefined";

    //std::cout << retString << " R: " << redAmount << " G: " << greenAmount << " B: " << blueAmount << std::endl;

    // Return
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

            // Test call
            cv::Mat croppedImage = image(cv::Rect(_tl.x, _tl.y, _br.x-_tl.x, _br.y-_tl.y));

            // Detect blobs
            std::vector<BlobColor> blob = filterAndFindBlobs(croppedImage);

            // Update global blob
            setBlobs(blob);

            // Check if any right blobs
            bool anyBricks = false;
            for(int i=0; i<blob.size(); i++)
            {
                std::string color = blob[i].color;
                if(color == "red" || color == "yellow" ||color == "blue")
                {
                    anyBricks = true;
                    break;
                }
            }

            std_msgs::Bool msg;
            if(anyBricks)
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

//! Projects a 2d polygon onto an 2d axis
/*!
    \param Axis defined by a vector
    \param Polygon as a set of points
    \param ref to Min value of Polygon intersection
    \param ref to Max value of Polygon intersection
*/
void projectPolygon(cv::Vec2f axis, cv::Point2f poly[4], float &min, float &max)
{
    // Use dot-product to project
    float projected = axis[0]*poly[0].x + axis[1] * poly[0].y;
    min = max = projected;

    for(unsigned int i=1; i<4; ++i)
    {
        // Use dot-product to project
        projected = axis[0]*poly[i].x + axis[1] * poly[i].y;

        if(projected < min)
            min = projected;

        if(projected > max)
            max = projected;
    }
}

//! Checks if two OBB is intersecting
/*!
    Uses the Separating Axis Theorem to compute if they are intersecting
    \param first OBB
    \param Second OBB
    \return True if the two OBB is intersecting, else false
*/
bool isRotatedRectIntersecting(cv::RotatedRect a, cv::RotatedRect b)
{
    // Points of rectangles
    cv::Point2f aPoints[4];
    cv::Point2f bPoints[4];
    a.points(aPoints);
    b.points(bPoints);

    // Check all non parallel edges of the OBB
    for(unsigned int i=0; i<4; ++i)
    {
        // Define edge
        cv::Point2f p1 = aPoints[i];
        cv::Point2f p2 = aPoints[i+1];

        // Find the axis perpendicular to the current edge.
        cv::Vec2f normal(p2.y-p1.y, p1.x-p2.x);

        // Project both polygons on that axis and Find the 2 outer points of both projections
        float minA = 0;
        float maxA = 0;
        projectPolygon(normal, aPoints, minA, maxA);

        float minB = 0;
        float maxB = 0;
        projectPolygon(normal, bPoints, minB, maxB);

        // Test if projection overlap
        if(maxA < minB || maxB < minA)
        {
            // If projections don't overlap, the polygons don't intersect
            return false;
        }
    }

    return true;
}

//! Finds which bricks can be grasped without collision with other bricks
/*!
    \param vector of found bricks in the image.
    \param Grasp width.
    \param width of grasp fingers.
    \return vector of collision free bricks
*/
std::vector<Brick> findNoGraspCollisionBricks(std::vector<Brick> brickVec,std::vector<cv::RotatedRect> monsterRectVec, float graspWidthPix, float fingerWidthPix)
{
    // Return var
    std::vector<Brick> freeBricksVec;

    // Iterate and check for all bricks
    for(std::vector<Brick>::iterator brickIt= brickVec.begin(); brickIt != brickVec.end(); ++brickIt)
    {
        // Define OBB which encapsulates the grasp
        cv::RotatedRect graspBox = brickIt->box;

        graspBox.angle = brickIt->theta;

        if(graspBox.size.height > graspBox.size.width)
        {
            // Height of box equals the finger width
            graspBox.size.height = fingerWidthPix;
            graspBox.size.width = graspWidthPix;
        }
        else
        {
            // Height of box equals the grasp width
            graspBox.size.height = graspWidthPix;
            graspBox.size.width = fingerWidthPix;
        }

        bool brickisIntersecting = false;

        // Check the OBB against all bricks
        for(std::vector<Brick>::iterator brickIt2= brickVec.begin(); brickIt2 != brickVec.end(); ++brickIt2)
        {
            // Dont check against the brick which should be grasped
            if(brickIt != brickIt2)
            {
                // Check for intersections
                if(isRotatedRectIntersecting(graspBox,brickIt2->box))
                {
                    brickisIntersecting = true;
                    break;
                }
            }
        }

        // Check the OBB against all monster rectangels
        for(std::vector<cv::RotatedRect>::iterator monstRectIt= monsterRectVec.begin(); monstRectIt != monsterRectVec.end(); ++monstRectIt)
        {
            // Check for intersections
            if(isRotatedRectIntersecting(graspBox,*monstRectIt))
            {
                brickisIntersecting = true;
                break;
            }
        }

        if(!brickisIntersecting)
        {
            // Add brick to vector of bricks which a free to grasp without collision.
            freeBricksVec.push_back(*brickIt);
        }
    }

    return freeBricksVec;
}

std::vector<Brick> findBricks()
{
    // Get blobs
    std::vector<BlobColor> blobColorVec = getBlobs();

    // Return var
    std::vector<Brick> brickVec;

    // Container for monster rectangels.
    std::vector<cv::RotatedRect> monsterRectVec;

    if(blobColorVec.empty() == false)
    {
        // Get image
        cv::Mat img;
        _blobMutex.lock();
        _blobImage.copyTo(img);
        _blobMutex.unlock();

        // Make OBB's
        for(int k=0; k<blobColorVec.size(); k++)
        {
            cv::RotatedRect box = cv::minAreaRect(cv::Mat(blobColorVec[k].blob));
            std::string color = blobColorVec[k].color;
            int area = box.size.area();

            //std::cout << color << ": " << area << std::endl;

            if(color == "red" && _minRedSize < area && area < _maxRedSize ||
               color == "yellow" && _minYellowSize < area && area < _maxYellowSize ||
               color == "blue" && _minBlueSize < area && area < _maxBlueSize)
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
                brick.box = box;
                brick.color = color;
                brick.theta = angle;
                brick.posX = convertPixelToM(box.center, img).x;
                brick.posY = convertPixelToM(box.center, img).y;
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

                // Add to vector if within workspace
                int lineThickness = 2;
                if(-_xMax < brick.posX && brick.posX < _xMax && -_yMax < brick.posY && brick.posY < _yMax )
                    brickVec.push_back(brick);

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
            else
            {
                if(color != "tape")
                {
                    // Save monster rect in vector
                    monsterRectVec.push_back(box);

                    //printConsole("Monster");
                }
                //else
                    //printConsole("Tape");
            }
        }

        // Choose only bricks where there is a non colliding grasp path.
        brickVec = findNoGraspCollisionBricks(brickVec,monsterRectVec,_graspWidthM*_pixelToM,_fingerWidthM*_pixelToM);

        // Draw center of image
        cv::Point2i center(img.cols/2, img.rows/2);
        cv::circle(img, center, 3, cv::Scalar(255,255,255), 5);

        // Draw workspace box
        cv::Point2i tl(center.x - _xMax * _pixelToM, center.y -_yMax * _pixelToM);
        cv::Point2i br(center.x + _xMax * _pixelToM, center.y +_yMax * _pixelToM);
        cv::rectangle(img, tl, br, cv::Scalar(255,255,255), 3);

        // Draw pickable bricks
        for(std::vector<Brick>::iterator it=brickVec.begin(); it!=brickVec.end();++it)
        {
            cv::Point2f vertices[4];
            it->box.points(vertices);

            // Show correspondant color on image
            cv::Scalar color;
            if(it->color == "red")
                color = cv::Scalar(0,0,255);
            else if(it->color == "blue")
                color = cv::Scalar(255,0,0);
            else if(it->color== "green")
                color = cv::Scalar(0,255,0);
            else if(it->color == "yellow")
                color = cv::Scalar(0,255,255);
            else if(it->color == "white")
                color = cv::Scalar(255,255,255);

            // Draw box in image
            for(int i=0; i<4; ++i)
                cv::line(img, vertices[i]+_tl, vertices[(i + 1) % 4]+_tl, color, 5, CV_AA);
        }

        // Draw Monsters
        for(std::vector<cv::RotatedRect>::iterator it=monsterRectVec.begin(); it!=monsterRectVec.end();++it)
        {
            cv::Point2f vertices[4];
            it->points(vertices);

            // Show correspondant color on image
            cv::Scalar color = cv::Scalar(0,255,0); //green

            // Draw box in image
            for(int i=0; i<4; ++i)
                cv::line(img, vertices[i]+_tl, vertices[(i + 1) % 4]+_tl, color, 2, CV_AA);

            // Draw center in image
            it->center += _tl;
            cv::circle(img, it->center, 3, cv::Scalar(255,255,255), 5);
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
    /*else if(func == "area")
        _minLegoArea = value;*/
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
    ros::init(argc, argv, "RC_Vision");
    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");

    // Topic names
    std::string imageSub, anyBricksPub, hmiConsolePub, analyzeService, visionParamSub, imagePub;
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");
    pNh.param<std::string>("image_sub", imageSub, "/RC_Camera/image_rect_color");
    pNh.param<std::string>("any_brick_pub", anyBricksPub, "/rcVision/anyBricks");
    pNh.param<std::string>("getBricks_service", analyzeService, "/rcVision/getBricks");
    pNh.param<std::string>("visionParamSub", visionParamSub, "/rcHMI/visionParam");
    pNh.param<std::string>("visionImagePub", imagePub, "/rcVision/image");
    pNh.param<double>("xMax", _xMax, 0.13);
    pNh.param<double>("yMax", _yMax, 0.08);
    pNh.param<double>("fingerWidth_meter", _graspWidthM, 0.03);
    pNh.param<double>("graspWidth_meter", _fingerWidthM, 0.045);

    // Service
    ros::ServiceServer analyzeFrameService = nh.advertiseService(analyzeService, analyzeFrameCallback);

    // Subscribers
    image_transport::ImageTransport itImg(nh);
    image_transport::Subscriber subImg = itImg.subscribe(imageSub, 1, imageCallback, image_transport::TransportHints("compressed"));
    ros::Subscriber paramSub = nh.subscribe(visionParamSub, 10, paramCallback);

    // Publishers
    _anyBricksPub = nh.advertise<std_msgs::Bool>(anyBricksPub, 1);
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);
    _imagePub = itImg.advertise(imagePub, 1);

    // Init
    cv::Point2f center(_imageSize.width/2, _imageSize.height/2);
    _tl.x = 0;
    _tl.y = center.y -_yMax * _pixelToM;
    _br.x = center.x*2;
    _br.y = center.y +_yMax * _pixelToM;

    // Brick detection thread
    boost::thread detectionThread(detectBricksThread);

    // Sleep rate
    ros::Rate r(30);

    // ROS Spin: Handle callbacks
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    // Interrupt thread
    detectionThread.interrupt();

    // Return
    return 0;
}
