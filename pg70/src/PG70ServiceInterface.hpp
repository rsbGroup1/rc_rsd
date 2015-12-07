/**/
#ifndef PG70SERVICEINTERFACE_HPP
#define PG70SERVICEINTERFACE_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

#include "pg70/PG70State.h"
#include "pg70/Move.h"
#include "pg70/Open.h"
#include "pg70/Close.h"
#include "pg70/Home.h"
#include "pg70/Stop.h"

#include "ros/ros.h"
#include <string>


class PG70ServiceInterface {
public:
	PG70ServiceInterface(const std::string& service_name, double loopRate);

	bool run();

protected:
	virtual bool move(float pos) = 0;
	virtual bool open(float force) = 0;
	virtual bool close(float force) = 0;
	virtual bool home() = 0;
	virtual bool stop() = 0;


	virtual void loop() = 0;
	virtual void stopDriver() = 0;


	double getLoopRate();
	void publish(double q);
	void publish(const pg70::PG70State& state);

private:

	bool moveHandle(pg70::Move::Request& request, pg70::Move::Response& response);
	bool openHandle(pg70::Open::Request& request, pg70::Open::Response& response);
	bool closeHandle(pg70::Close::Request& request, pg70::Close::Response& response);
	bool homeHandle(pg70::Home::Request& request, pg70::Home::Response& response);
	bool stopHandle(pg70::Stop::Request& request, pg70::Stop::Response& response);

private:
	ros::NodeHandle _nodeHnd;

    ros::Publisher _statePublisher;
    ros::ServiceServer _srvMove;
    ros::ServiceServer _srvOpen;
    ros::ServiceServer _srvClose;
    ros::ServiceServer _srvHome;
    ros::ServiceServer _srvStop;

    std::string _service_name;
    ros::Rate _loopRate;

};

#endif //#ifndef PG70SERVICEINTERFACE_HPP
