#ifndef CNBIROS_CONTROL_NAVIGATIONHANDLER_CPP
#define CNBIROS_CONTROL_NAVIGATIONHANDLER_CPP

#include "cnbiros_state_control/NavigationHandler.hpp"

namespace cnbiros {
	namespace control {

NavigationHandler::NavigationHandler(void) : p_nh_("~") {


	this->configure();


	// Subcriber
	this->subctr_ = this->nh_.subscribe(this->ctrtopic_, 1, &NavigationHandler::on_received_state, this);

	// Service clients
	for(auto it=this->nav_services_.begin(); it!=this->nav_services_.end(); ++it) {
		this->srv_clients_[it->first] = this->nh_.serviceClient<std_srvs::Empty>(it->second);
		ROS_INFO("[NavigationHandler] - Configured services: ['%s'] => %s", it->first.c_str(), it->second.c_str());
	}
}

NavigationHandler::~NavigationHandler(void) {}

bool NavigationHandler::configure(void) {

	XmlRpc::XmlRpcValue dict;
	std::string	dict_key, dict_label;

	// Getting standard parameters
	this->p_nh_.param<std::string>("state_topic", this->ctrtopic_, "/control/system_state");
	
	// Getting dictionary for rules
	if(this->p_nh_.getParam("rules", dict) == false) {
		ROS_ERROR("[NavigationHandler] Cannot retrieve required rules dictionary.");
		return false;
	}
	
	try {
		for(auto i = 0; i<dict.size(); ++i) {
			if(dict[i].hasMember("state") == false || dict[i].hasMember("service") == false) {
				ROS_ERROR("Commands dictionary must have fields 'state' and 'service'");
				return false;
			}
			dict_key = static_cast<std::string>(dict[i]["state"]);
			this->nav_services_[dict_key] = static_cast<std::string>(dict[i]["service"]);
		}
	} catch (XmlRpc::XmlRpcException& ex) {
		ROS_ERROR("Wrong format for services dictionary.");
		return false;
	}


	return true;


}

void NavigationHandler::on_received_state(const cnbiros_state_control::SystemStateMsg& msg) {


	std_srvs::Empty srv;
	auto it = this->srv_clients_.find(msg.label);

	if( it != this->srv_clients_.end() ) {

		if(it->second.call(srv)) {
			ROS_DEBUG_NAMED("service_call", "[NavigationHandler] Service call for navigation '%s'", 
							it->first.c_str());
		} else {
			ROS_DEBUG_NAMED("service_call", "[NavigationHandler] No answer to the service call '%s'", 
							it->first.c_str());
		}

	}
		

}



	}
}


#endif
