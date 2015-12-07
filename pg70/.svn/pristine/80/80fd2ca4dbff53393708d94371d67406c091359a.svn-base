#include "PG70.hpp"

using namespace rwhw;
using namespace rw::math;
using namespace rw::common;

PG70::PG70(const PropertyMap& properties) : PG70ServiceInterface(properties.get<std::string>("Name"), 20), _properties(properties)
{
    if (_pg70.connectSerial(properties.get<std::string>("Port")))
		std::cout<<"Connect to PG70 Finished"<<std::endl;
    else
        std::cerr<<"Connect to PG70 Failed"<<std::endl;
}

PG70::PG70(std::string serialPort) : PG70ServiceInterface("PG70", 20)
{
    if (_pg70.connectSerial(serialPort))
        std::cout<<"Connect to PG70 Finished"<<std::endl;
    else
        std::cerr<<"Connect to PG70 Failed!"<<std::endl;
}

void PG70::stopDriver()
{

}

void PG70::loop()
{
	Q q(1);
	if (_pg70.getQ(q))
		publish(q(0));
}

bool PG70::move(float pos)
{
	std::cout<<"Move to "<<pos<<std::endl;
	_pg70.setQ(Q(1, pos));
	return true;
}

bool PG70::open(float force)
{
	std::cout<<"Open with "<<force<<std::endl;
	_pg70.setGraspPowerPct(force);
	_pg70.open();
	return true;
}

bool PG70::close(float force)
{
	std::cout<<"Close with "<<force<<std::endl;
	_pg70.setGraspPowerPct(force);
	_pg70.close();
	return true;
}

bool PG70::home()
{
	std::cout<<"Home"<<std::endl;
	_pg70.home();
	return true;
}

bool PG70::stop()
{
	std::cout<<"Stop"<<std::endl;
	_pg70.stop();
	return true;
}

