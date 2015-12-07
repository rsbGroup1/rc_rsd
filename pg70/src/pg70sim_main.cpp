#include "PG70Sim.hpp"
#include <boost/bind.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>

using namespace rw::common;
using namespace rw::models;
using namespace rw::loaders;
using namespace rw::kinematics;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ursim");


   // PG70Sim pg70(argc, argv);



    //pg70.run();

    
//  while (ros::ok())
//  {
//    /**
//     * This is a message object. You stuff it with data, and then publish it.
//     */
//    sensor_data msg;
//
//    std::stringstream ss;
//    ss << "hello";// << count;
//    msg.name = ss.str();
//
//    ROS_INFO("%s", msg.name.c_str());
//
//    /**
//     * The publish() function is how you send messages. The parameter
//     * is the message object. The type of this object must agree with the type
//     * given as a template parameter to the advertise<>() call, as was done
//     * in the constructor above.
//     */
//    chatter_pub.publish(msg);
//
//    ros::spinOnce();
//
//    loop_rate.sleep();
//    ++count;
//  }



    return 0;
}

