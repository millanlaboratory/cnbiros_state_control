#ifndef CNBIROS_STATE_CONTROL_BCISTATE_HPP
#define CNBIROS_STATE_CONTROL_BCISTATE_HPP

// System includes
#include <map>

// Ros includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <xmlrpcpp/XmlRpc.h>
#include <XmlRpcValue.h>

// Package includes
#include <cnbiros_bci/TidMessage.h>

namespace cnbiros {
	namespace control {

class BciState {

	public:
		BciState(void);
		virtual ~BciState(void);

		bool configure(void);

	private:
		void on_received_tid(const cnbiros_bci::TidMessage& msg);


	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;

		ros::Subscriber	sub_;
		ros::ServiceClient	srv_start_;
		ros::ServiceClient	srv_stop_;

		std::string		stopic_;

		std::map<std::string, std::string>			services_names_;
		std::map<std::string, ros::ServiceClient>	services_;

};


	}
}

#endif
