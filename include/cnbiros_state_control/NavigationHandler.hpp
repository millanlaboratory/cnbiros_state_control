#ifndef CNBIROS_CONTROL_NAVIGATIONHANDLER_HPP
#define CNBIROS_CONTROL_NAVIGATIONHANDLER_HPP

// System includes
#include <unordered_map>

// Ros includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <xmlrpcpp/XmlRpc.h>
#include <XmlRpcValue.h>

// Package include
#include "cnbiros_state_control/SystemStateMsg.h"
#include "cnbiros_state_control/SystemState.hpp"
#include "cnbiros_state_control/SystemStateConverter.hpp"

namespace cnbiros {
	namespace control {

class NavigationHandler {

	public:
		NavigationHandler(void);
		virtual ~NavigationHandler(void);

		bool configure(void);

	private:
		void on_received_state(const cnbiros_state_control::SystemStateMsg& msg);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;

		ros::Subscriber		subctr_;


		std::string		ctrtopic_;


		std::unordered_map<std::string, ros::ServiceClient> srv_clients_;
		std::unordered_map<std::string, std::string> nav_services_;
		bool is_enable_;



};

	}
}

#endif

