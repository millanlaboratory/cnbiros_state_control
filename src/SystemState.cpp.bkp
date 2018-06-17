#ifndef CNBIROS_STATE_CONTROL_SYSTEMSTATE_CPP
#define CNBIROS_STATE_CONTROL_SYSTEMSTATE_CPP

#include "cnbiros_state_control/SystemState.hpp"

namespace cnbiros {
	namespace control {

SystemState::SystemState(void) : p_nh_("~") {



	this->srv_joystick_control_   = this->p_nh_.advertiseService("joystick_control", &SystemState::on_joystick_control, this);
	this->srv_navigation_control_ = this->p_nh_.advertiseService("navigation_control", &SystemState::on_navigation_control, this);
	this->srv_navigation_start_   = this->p_nh_.advertiseService("navigation_start", &SystemState::on_navigation_start, this);
	this->srv_navigation_stop_    = this->p_nh_.advertiseService("navigation_stop", &SystemState::on_navigation_stop, this);


	this->cli_joystick_enable_    = this->nh_.serviceClient<std_srvs::Empty>("/joy_filter_control/joystick_enable");
	this->cli_joystick_disable_   = this->nh_.serviceClient<std_srvs::Empty>("/joy_filter_control/joystick_disable");
	this->cli_navigation_enable_  = this->nh_.serviceClient<std_srvs::Empty>("/shared_dynamics/navigation_enable");
	this->cli_navigation_disable_ = this->nh_.serviceClient<std_srvs::Empty>("/shared_dynamics/navigation_disable");
	this->cli_navigation_start_   = this->nh_.serviceClient<std_srvs::Empty>("/shared_dynamics/navigation_start");
	this->cli_navigation_stop_    = this->nh_.serviceClient<std_srvs::Empty>("/shared_dynamics/navigation_stop");


	this->state_ = State::Navigation;
}

SystemState::~SystemState(void) {}

bool SystemState::IsState(State state) {

	if(this->state_ == state)
		return true;

	return false;
}

bool SystemState::on_joystick_control(std_srvs::Empty::Request& req,
									   std_srvs::Empty::Response& res) {

	if(this->IsState(State::Joystick))
			return true;

	// Stopping navigation
	this->request_navigation_stop();

	// Disable navigation
	this->request_navigation_disable();

	// Enable joystick
	this->request_joystick_enable();

	this->state_ = State::Joystick;

	return true;
}

bool SystemState::on_navigation_control(std_srvs::Empty::Request& req,
											  std_srvs::Empty::Response& res) {

	bool execute = true;

	switch(this->state_) {
		case State::Navigation:
		case State::Running:
		case State::Stopped:
			execute = false;
			break;
	}

	if(execute == false)
		return true;

	// Disable joystick
	this->request_joystick_disable();

	// Enable navigation
	this->request_navigation_enable();

	// Start navigation
	this->request_navigation_start();
	
	this->state_ = State::Running;
	
	return true;
}

bool SystemState::on_navigation_stop(std_srvs::Empty::Request& req,
									  std_srvs::Empty::Response& res) {

	if(this->IsState(State::Running) == false)
		return true;

	// Stopping navigation
	this->request_navigation_stop();

	this->state_ = State::Stopped;
	
	return true;
}

bool SystemState::on_navigation_start(std_srvs::Empty::Request& req,
									  std_srvs::Empty::Response& res) {

	bool execute = true;

	switch(this->state_) {
		case State::Idle:
		case State::Joystick:
		case State::Running:
			execute = false;
			break;
	}

	if(execute == false)
		return true;

	// Stopping navigation
	this->request_navigation_start();
	
	this->state_ = State::Running;

	return true;
}


void SystemState::request_navigation_stop(void) {
	
	std_srvs::Empty srv;

	if(this->cli_navigation_stop_.call(srv))
		ROS_WARN("[SystemState] - Navigation requested to stop");
	else
		ROS_ERROR("[SystemState] - Failed to request navigation to stop");
}

void SystemState::request_navigation_start(void) {
	
	std_srvs::Empty srv;


	if(this->cli_navigation_start_.call(srv))
		ROS_WARN("[SystemState] - Navigation requested to start");
	else
		ROS_ERROR("[SystemState] - Failed to request navigation to start");
}

void SystemState::request_navigation_enable(void) {
	
	std_srvs::Empty srv;


	if(this->cli_navigation_enable_.call(srv))
		ROS_WARN("[SystemState] - Navigation requested to be enabled");
	else
		ROS_ERROR("[SystemState] - Failed to request navigation to be enabled");
}

void SystemState::request_navigation_disable(void) {
	
	std_srvs::Empty srv;


	if(this->cli_navigation_disable_.call(srv))
		ROS_WARN("[SystemState] - Navigation requested to be disabled");
	else
		ROS_ERROR("[SystemState] - Failed to request navigation to be disabled");
}

void SystemState::request_joystick_enable(void) {
	
	std_srvs::Empty srv;


	if(this->cli_joystick_enable_.call(srv))
		ROS_WARN("[SystemState] - Joystick requested to be enabled");
	else
		ROS_ERROR("[SystemState] - Failed to request joystick to be enabled");
}

void SystemState::request_joystick_disable(void) {
	
	std_srvs::Empty srv;


	if(this->cli_joystick_disable_.call(srv))
		ROS_WARN("[SystemState] - Joystick requested to be disabled");
	else
		ROS_ERROR("[SystemState] - Failed to request joystick to be disabled");
}

	}
}


#endif
