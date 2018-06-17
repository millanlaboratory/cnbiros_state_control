#ifndef CNBIROS_STATE_CONTROL_SYSTEMMANAGER_HPP
#define CNBIROS_STATE_CONTROL_SYSTEMMANAGER_HPP

// System includes
#include <unordered_map>

// Ros includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <xmlrpcpp/XmlRpc.h>
#include <XmlRpcValue.h>

// Package includes
#include "cnbiros_state_control/SystemState.hpp"
#include "cnbiros_state_control/SystemStateConverter.hpp"


namespace cnbiros {
	namespace control {

class SystemManager {

	public:
		SystemManager(void);
		virtual ~SystemManager(void);

		bool configure(void);

	private:
		bool on_received_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res, std::string caller);

		bool configure_states(void);
		bool configure_transitions(void);
		bool add_service(std::string label, std::string name);
	private:
		ros::NodeHandle	nh_;
		ros::NodeHandle	private_nh_;

		ros::Publisher	pub_;
		std::string		ptopic_;

		std::unordered_map<std::string, ros::ServiceServer>		services_map_;
		std::unordered_map<std::string, std::string>		names_map_;

		SystemState		state_;

};


	}
}


#endif
