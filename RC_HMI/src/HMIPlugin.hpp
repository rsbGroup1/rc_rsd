#ifndef HMIPLUGIN_HPP
#define HMIPLUGIN_HPP

// Includes
#include <rws/RobWorkStudioPlugin.hpp>
#include "HMIWidget.hpp"

class HMIPlugin : public rws::RobWorkStudioPlugin
{
	Q_OBJECT
        Q_INTERFACES(rws::RobWorkStudioPlugin)

	public:
        HMIPlugin();
        virtual ~HMIPlugin();
        virtual void open(rw::models::WorkCell* workcell);
        virtual void close();
        virtual void initialize();

	private slots:
        void stateChangedListener(const rw::kinematics::State &state);

	private:
        HMIWidget* _hmiWidget;
        void genericEvent(const std::string& str);
};
#endif // HMIPLUGIN_HPP
