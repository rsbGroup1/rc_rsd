// Includes
#include "HMIPlugin.hpp"

void AddIPL98ErrorHistory(char const *)
{
	//
}

HMIPlugin::HMIPlugin() : RobWorkStudioPlugin("HMI", QIcon("/home/student/catkin_ws/src/RSD_HMI/images/iconhmi2.png"))
{
   	QWidget *widg = new QWidget(this);
	QVBoxLayout *lay = new QVBoxLayout(widg);
	widg->setLayout(lay);
	this->setWidget(widg);

    // Setup ToolBar
    _hmiWidget = new HMIWidget(this);
    lay->addWidget(_hmiWidget);
}

HMIPlugin::~HMIPlugin()
{
	//
}

void HMIPlugin::initialize()
{
        getRobWorkStudio()->genericEvent().add(boost::bind(&HMIPlugin::genericEvent, this, _1), this);
        getRobWorkStudio()->stateChangedEvent().add(boost::bind(&HMIPlugin::stateChangedListener, this, _1), this);
}

void HMIPlugin::stateChangedListener(const rw::kinematics::State &state)
{
    _hmiWidget->stateChangedListener(state);
}

void HMIPlugin::genericEvent(const std::string& str)
{
 	//
}

void HMIPlugin::open(rw::models::WorkCell* workcell)
{
    _hmiWidget->initialize(workcell, getRobWorkStudio());
}   

void HMIPlugin::close()
{
	//
}

Q_EXPORT_PLUGIN(HMIPlugin);
