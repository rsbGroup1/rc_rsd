/**/
#include "PG70ServiceInterface.hpp"

#include <rw/math.hpp>
#include <boost/foreach.hpp>


using namespace pg70;

using namespace rw::common;
using namespace rw::math;


PG70ServiceInterface::PG70ServiceInterface(const std::string& service_name, double loopRate):
	_service_name(service_name),
	_loopRate(loopRate)
{
    //_sensor_data_publisher = _nodeHnd.advertise<sensor_data>("sensor_data", 1000);

    _statePublisher = _nodeHnd.advertise<PG70State>(_service_name+"/PG70State", 5);

    _srvMove = _nodeHnd.advertiseService(_service_name+"/move", &PG70ServiceInterface::moveHandle, this);
	_srvOpen = _nodeHnd.advertiseService(_service_name+"/open", &PG70ServiceInterface::openHandle, this);
	_srvClose = _nodeHnd.advertiseService(_service_name+"/close", &PG70ServiceInterface::closeHandle, this);
	_srvHome = _nodeHnd.advertiseService(_service_name+"/home", &PG70ServiceInterface::homeHandle, this);
	_srvStop = _nodeHnd.advertiseService(_service_name+"/stop", &PG70ServiceInterface::stopHandle, this);


}


bool PG70ServiceInterface::moveHandle(Move::Request& request, Move::Response& response) {
	ROS_INFO("Move %f", request.pos);
	return move(request.pos);
}


bool PG70ServiceInterface::openHandle(pg70::Open::Request& request, pg70::Open::Response& response) {
	ROS_INFO("Open %f", request.power);
	return open(request.power);
}

bool PG70ServiceInterface::closeHandle(pg70::Close::Request& request, pg70::Close::Response& response) {
	ROS_INFO("Close %f", request.power);
	return close(request.power);
}

bool PG70ServiceInterface::homeHandle(pg70::Home::Request& request, pg70::Home::Response& response) {
	ROS_INFO("home");
	return home();
}

bool PG70ServiceInterface::stopHandle(pg70::Stop::Request& request, pg70::Stop::Response& response) {
	ROS_INFO("stop");
	return stop();
}



void PG70ServiceInterface::publish(double q) {
	PG70State state;
	state.name = _service_name;
	state.pos = q;
	publish(state);
}

void PG70ServiceInterface::publish(const PG70State& state) {
	_statePublisher.publish(state);
}


bool PG70ServiceInterface::run() {

	  while (ros::ok())
	  {
		  loop();
		  ros::spinOnce();
		  _loopRate.sleep();
	  }
	  stopDriver();
	  return true;
}

double PG70ServiceInterface::getLoopRate() {
	return _loopRate.cycleTime().toSec();
}
