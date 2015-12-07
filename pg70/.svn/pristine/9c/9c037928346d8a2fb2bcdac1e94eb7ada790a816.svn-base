/**/
#ifndef PG70_HPP
#define PG70_HPP

#include "PG70ServiceInterface.hpp"
#include <rw/common/PropertyMap.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>
#include <rwhw/schunkpg70/SchunkPG70.hpp>
#include <vector>

class PG70: public PG70ServiceInterface 
{
public:
	PG70(const rw::common::PropertyMap& properties);
    PG70(std::string serialPort);

protected:
	virtual void loop();
	virtual void stopDriver();

	virtual bool move(float pos);
	virtual bool open(float force);
	virtual bool close(float force);
	virtual bool home();
	virtual bool stop();

private:
	rwhw::SchunkPG70 _pg70;
	rw::common::PropertyMap _properties;

};

#endif //#ifndef UNIVERSALROBOTS_HPP
