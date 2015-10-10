// Includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <iostream>
#include <rc_plc/MoveConv.h>
#include <rc_plc/StopConv.h>
#include <rc_plc/StartConv.h>
#include "serial/serial.h"

// Global variables
ros::Publisher _hmiConsolePub;

// Functions
void printConsole(std::string msg)
{
    ROS_ERROR_STREAM(msg.c_str());
    std_msgs::String pubMsg;
    pubMsg.data = "PLC: " + msg;
    _hmiConsolePub.publish(pubMsg);
}

bool moveCallback(rc_plc::MoveConv::Request &req, rc_plc::MoveConv::Response &res)
{
    //
}

bool startCallback(rc_plc::StartConv::Request &req, rc_plc::StartConv::Response &res)
{
    //
}

bool stopCallback(rc_plc::StopConv::Request &req, rc_plc::StopConv::Response &res)
{
    //
}

/*void writeSerialThread()
{
    while(true)
    {
        try
        {
            // Write
            _serialConnection->write(_queue.dequeue());

            // Signal interrupt point
            boost::this_thread::interruption_point();
        }
        catch(const boost::thread_interrupted&)
        {
            break;
        }
    }
}

bool compareMsg(char* msg, char* command)
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
    ros::NodeHandle pNh(ros::this_node::getName() + "/");

    // Topic names
    std::string hmiConsolePub;
    pNh.param<std::string>("hmiConsole", hmiConsolePub, "/rcHMI/console");

    // Publishers
    _hmiConsolePub = nh.advertise<std_msgs::String>(hmiConsolePub, 100);

    // Create service
    ros::ServiceServer serviceMove = nh.advertiseService("/rcPLC/MoveConv", moveCallback);
    ros::ServiceServer serviceStart = nh.advertiseService("/rcPLC/StartConv", startCallback);
    ros::ServiceServer serviceStop = nh.advertiseService("/rcPLC/StopConv", stopCallback);

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
