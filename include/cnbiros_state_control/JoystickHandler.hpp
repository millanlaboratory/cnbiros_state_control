#ifndef CNBIROS_CONTROL_JOYSTICKHANDLER_HPP
#define CNBIROS_CONTROL_JOYSTICKHANDLER_HPP

// System includes
#include <map>

// Ros includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Package include
#include "cnbiros_state_control/SystemStateMsg.h"
#include "cnbiros_state_control/SystemState.hpp"
#include "cnbiros_state_control/SystemStateConverter.hpp"

namespace cnbiros {
	namespace control {

class JoystickHandler {

	public:
		JoystickHandler(void);
		virtual ~JoystickHandler(void);

		bool configure(void);

	private:
		void on_received_joy_cmdvel(const geometry_msgs::Twist& msg);
		void on_received_state(const cnbiros_state_control::SystemStateMsg& msg);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;

		ros::Subscriber	subctr_;
		ros::Subscriber	subjoy_;
		ros::Publisher  pubvel_;

		std::string		ctrtopic_;
		std::string		joytopic_;
		std::string		veltopic_;

		bool is_enable_;



};

	}
}

#endif
