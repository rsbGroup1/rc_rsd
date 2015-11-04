// Includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <iostream>
#include <rc_plc/MoveConv.h>
#include <rc_plc/StopConv.h>
#include <rc_plc/StartConv.h>
#include <rc_plc/ChangeDirection.h>
#include "serial/serial.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <queue>

// Defines
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()
#define DATA_LENGTH             10

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
};

// Global var
serial::Serial *_serialConnection;
bool _debugMsg;
SynchronisedQueue<std::string> _queue;
boost::thread *_writeThread;
ros::Publisher _hmiConsolePub;

// Functions
void printConsole(std::string msg)
{
    ROS_ERROR_STREAM(msg.c_str());
    std_msgs::String pubMsg;
    pubMsg.data = "PLC: " + msg;
    _hmiConsolePub.publish(pubMsg);
}

bool changeDirectionCallback(rc_plc::ChangeDirection::Request &req, rc_plc::ChangeDirection::Response &res)
{
    //true = reverse. false = forward.
    if(req.direction)
        _queue.enqueue("r");
    else
        _queue.enqueue("s");

    return true;
}

bool moveCallback(rc_plc::MoveConv::Request &req, rc_plc::MoveConv::Response &res)
{
    if(req.direction)
        _queue.enqueue("r");
    else
        _queue.enqueue("s");
    sleep(req.duration);
    _queue.enqueue("t");
    return true;
}

bool startCallback(rc_plc::StartConv::Request &req, rc_plc::StartConv::Response &res)
{
    if(req.direction)
        _queue.enqueue("r");
    else
        _queue.enqueue("s");
    return true;
}

bool stopCallback(rc_plc::StopConv::Request &req, rc_plc::StopConv::Response &res)
{
    _queue.enqueue("t");
    return true;
}

void writeSerialThread()
{
    while(true)
    {
        try
        {
            if(_queue.size()>0)
            {
                // Write
                _serialConnection->write(_queue.dequeue());
            }

            // Signal interrupt point
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            boost::this_thread::interruption_point();
        }
        catch(const boost::thread_interrupted&)
        {
            break;
        }
    }
}

/*bool compareMsg(char* msg, char* command)
{
    int i = 0;
    while(msg[i] != '\n' && command[i] != '\n')
    {
        if(tolower(msg[i]) != tolower(command[i]))
            return false;

        i++;
    }

    if(i == 0)
        return false;

    return true;
}

void readSerialThread()
{
    std::string tempString;
    char msg[DATA_LENGTH+1];
    msg[DATA_LENGTH] = '\n';
    int i = 0;

    while(true)
    {
        _serialMutex.lock();
        tempString = _serialConnection->read(1);
        _serialMutex.unlock();

        if(tempString.size() == 1)
        {
            msg[i] = tempString[0];

            if(msg[i] == '\n')
            {
                if(compareMsg(msg, "start\n") || compareMsg(msg, "stop\n"))
                {
                    std::string stringMsg(msg);
                    stringMsg = stringMsg.substr(0, stringMsg.size()-1);
                    std_msgs::String topicMsg;
                    topicMsg.data = stringMsg;
                    _publishMutex.lock();
                    _missionPlannerPublisher.publish(topicMsg);
                    _publishMutex.unlock();

                    std::cout << "Publishing: " << stringMsg << std::endl;
                }

                // Clear data
                i = 0;
            }
            else	// Wait for new character
                i++;
        }
        else
        {
            // Clear if buffer is full without newline
            if(i == DATA_LENGTH-1)
            {
                i = 0;
                for(int k=0; k<DATA_LENGTH; k++)
                    msg[k] = ' ';
                msg[DATA_LENGTH] = '\n';
            }
        }
    }

    // Close connection
    _serialConnection->close();
}*/

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "rc_plc");
    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");

    // Topic names
    std::string hmiConsolePub, port;
    int baudRate;
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");
    pNh.param<int>("baud_rate", baudRate, 19200);
    pNh.param<std::string>("port", port, "/dev/serial/by-id/usb-FTDI_USB-RS485_Cable_FTVFYA1U-if00-port0");

    // Publishers
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);

    // Create service
    ros::ServiceServer serviceMove = nh.advertiseService("/rcPLC/MoveConv", moveCallback);
    ros::ServiceServer serviceStart = nh.advertiseService("/rcPLC/StartConv", startCallback);
    ros::ServiceServer serviceStop = nh.advertiseService("/rcPLC/StopConv", stopCallback);
    ros::ServiceServer serviceChange = nh.advertiseService("/rcPLC/ChangeDirection", changeDirectionCallback);

    // Open connection
    _serialConnection = new serial::Serial(port.c_str(), baudRate, serial::Timeout::simpleTimeout(50));

    // Check if connection is ok
    if(!_serialConnection->isOpen())
    {
        printConsole("Error opening connection!");
        _serialConnection->close();
        return 0;
    }

    // Start serial read thread
    _writeThread = new boost::thread(writeSerialThread);

    // ROS Spin: Handle callbacks
    while(ros::ok())
        ros::spinOnce();

    // Close connection
    _writeThread->interrupt();
    _serialConnection->close();

    // Return
    return 0;
}
