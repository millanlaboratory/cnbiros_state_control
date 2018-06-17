#ifndef CNBIROS_STATE_CONTROL_SYSTEMSTATECONVERTER_HPP
#define CNBIROS_STATE_CONTROL_SYSTEMSTATECONVERTER_HPP

#include "cnbiros_state_control/SystemState.hpp"
#include "cnbiros_state_control/SystemStateMsg.h"


namespace cnbiros {
	namespace control {

class SystemStateConverter {

	public:

		static bool FromMessage(const cnbiros_state_control::SystemStateMsg& msg, SystemState& state);
		static bool ToMessage(const SystemState& state, cnbiros_state_control::SystemStateMsg& msg);

};


	}
}

#endif
