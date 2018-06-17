#ifndef CNBIROS_CONTROL_JOYSTICKHANDLER_CPP
#define CNBIROS_CONTROL_JOYSTICKHANDLER_CPP

#include "cnbiros_state_control/JoystickHandler.hpp"

namespace cnbiros {
	namespace control {

JoystickHandler::JoystickHandler(void) : p_nh_("~") {
	
	this->configure();

	this->subctr_ = this->nh_.subscribe(this->ctrtopic_, 1, &JoystickHandler::on_received_state, this);
	this->subjoy_ = this->nh_.subscribe(this->joytopic_, 1, &JoystickHandler::on_received_joy_cmdvel, this);
	this->pubvel_ = this->nh_.advertise<geometry_msgs::Twist>(this->veltopic_, 1);
}

JoystickHandler::~JoystickHandler(void) {}

bool JoystickHandler::configure(void) {

	this->is_enable_ = false;

	this->ctrtopic_ = "/control/system_state";
	this->joytopic_ = "/joy_teleop/cmd_vel";
	this->veltopic_ = "/cmd_vel";

	return true;
}

void JoystickHandler::on_received_state(const cnbiros_state_control::SystemStateMsg& msg) {

	if(msg.label.compare("Joystick") == 0) {
		this->is_enable_ = true;
		ROS_WARN("[JoystickHandler] Joystick control enabled");
	} else {
		this->is_enable_ = false;
		ROS_WARN("[JoystickHandler] Joystick control disabled");
	}
}

void JoystickHandler::on_received_joy_cmdvel(const geometry_msgs::Twist& msg) {
	
	if(this->is_enable_ == true)
		this->pubvel_.publish(msg);
}

	}
}


#endif
