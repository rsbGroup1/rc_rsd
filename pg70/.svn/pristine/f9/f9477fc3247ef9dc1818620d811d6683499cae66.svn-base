/**/
#ifndef PG70SIM_HPP
#define PG70SIM_HPP

#include "PG70ServiceInterface.hpp"

#include <rw/kinematics/State.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/trajectory/Trajectory.hpp>

class PG70Sim: public PG70ServiceInterface {
public:
	PG70Sim(int argc, char** argv);

protected:
	virtual bool move(float pos) ;
	virtual bool grip(float force);
	virtual bool stop();
	virtual bool pause();
	virtual bool start();
	virtual bool wait();

	virtual void loop();
	virtual void stopDriver();

private:

};

#endif //#ifndef PG70SIM_HPP
