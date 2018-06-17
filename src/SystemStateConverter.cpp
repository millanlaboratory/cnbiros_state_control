#ifndef CNBIROS_STATE_CONTROL_SYSTEMSTATECONVERTER_CPP
#define CNBIROS_STATE_CONTROL_SYSTEMSTATECONVERTER_CPP

#include "cnbiros_state_control/SystemStateConverter.hpp"


namespace cnbiros {
	namespace control {


bool SystemStateConverter::FromMessage(const cnbiros_state_control::SystemStateMsg& msg, SystemState& state) {

	return state.Set(msg.id);
}

bool SystemStateConverter::ToMessage(const SystemState& state, cnbiros_state_control::SystemStateMsg& msg) {

	SystemState		cstate(state);
	msg.id			= cstate.GetId();
	msg.label		= cstate.GetLabel();
	msg.prev_id		= cstate.GetPrevId();
	msg.prev_label	= cstate.GetPrevLabel();
	
	return true;
}

	}
}

#endif
