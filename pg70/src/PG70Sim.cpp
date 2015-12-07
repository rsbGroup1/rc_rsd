/**/
#include "PG70Sim.hpp"

#include <rw/math/MetricFactory.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::trajectory;

PG70Sim::PG70Sim(int argc, char** argv):
	PG70ServiceInterface("PG70", 10 )
{


}

void PG70Sim::stopDriver() {

}

void PG70Sim::loop()
{
}

bool PG70Sim::move(float pos)
{
	return true;
}

bool PG70Sim::grip(float force)
{
	return true;
}

bool PG70Sim::stop()
{
	return true;
}


bool PG70Sim::pause()
{
	return true;
}

bool PG70Sim::start()
{
	return true;
}

bool PG70Sim::wait()
{
	return true;
}
