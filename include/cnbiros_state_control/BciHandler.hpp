#ifndef CNBIROS_CONTROL_BCIHANDLER_HPP
#define CNBIROS_CONTROL_BCIHANDLER_HPP

// System includes
#include <map>
#include <sstream>

// Ros includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <xmlrpcpp/XmlRpc.h>
#include <XmlRpcValue.h>

// Package includes
#include <cnbiros_bci/TidMessage.h>
#include "cnbiros_state_control/SystemStateMsg.h"

namespace cnbiros {
	namespace control {

class BciHandler {

	public:
		BciHandler(void);
		virtual ~BciHandler(void);

		bool configure(void);

	private:
		void on_received_tid(const cnbiros_bci::TidMessage& msg);
		void on_received_state(const cnbiros_state_control::SystemStateMsg& msg);


	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;

		ros::Subscriber	sub_;
		ros::Subscriber subctr_;
		ros::Publisher	pub_;
		ros::ServiceClient	srv_start_;
		ros::ServiceClient	srv_stop_;

		std::string		stopic_;
		std::string		ctrtopic_;
		std::string		pubtopic_;

		std::map<std::string, std::string>			services_names_;
		std::map<std::string, std::string>			events_map_;
		std::map<std::string, ros::ServiceClient>	services_;

};


	}
}

#endif
