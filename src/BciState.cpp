#ifndef CNBIROS_STATE_CONTROL_BCISTATE_CPP
#define CNBIROS_STATE_CONTROL_BCISTATE_CPP

#include "cnbiros_state_control/BciState.hpp"


namespace cnbiros {
	namespace control {

BciState::BciState(void) : p_nh_("~") {

	this->configure();

	// Initialize subscriber
	this->sub_ = this->nh_.subscribe(this->stopic_, 1, &BciState::on_received_tid, this);

	// Initialize services
	for(auto it=this->services_names_.begin(); it!=this->services_names_.end(); ++it) {
		this->services_[it->first] = this->nh_.serviceClient<std_srvs::Empty>(it->second);
		ROS_INFO("Configured services: ['%s'] => %s", it->first.c_str(), it->second.c_str());
	}

}

BciState::~BciState(void) {}

bool BciState::configure(void) {
	
	XmlRpc::XmlRpcValue dict;
	std::string	dict_key;

	this->stopic_ = "/rostid_cnbi2ros";

	// Getting dictionary for the events<=>angles association
	if(this->p_nh_.getParam("states", dict) == false) {
		ROS_ERROR("Cannot retrieve required states dictionary.");
		return false;
	}
	
	try {
		for(auto i = 0; i<dict.size(); ++i) {
			if(dict[i].hasMember("event") == false || dict[i].hasMember("service") == false) {
				ROS_ERROR("Commands dictionary must have fields 'event' and 'service'");
				return false;
			}
			dict_key = static_cast<std::string>(dict[i]["event"]);
			this->services_names_[dict_key] = static_cast<std::string>(dict[i]["service"]);
		}
	} catch (XmlRpc::XmlRpcException& ex) {
		ROS_ERROR("Wrong format for services dictionary.");
		return false;
	}


	return true;
}

void BciState::on_received_tid(const cnbiros_bci::TidMessage& msg) {

	std_srvs::Empty srv;
	std::stringstream sevent;
	std::map<std::string, ros::ServiceClient>::iterator it;
	std::map<std::string, std::string>::iterator itn;

	sevent << std::showbase << std::hex << msg.event;

	it  = this->services_.find(sevent.str());
	itn = this->services_names_.find(sevent.str()); 

	if(it != this->services_.end() && itn != this->services_names_.end()) {

		if(it->second.call(srv)) {
			ROS_DEBUG_NAMED("service_call", "Service call for service '%s'=>'%s'", 
							itn->first.c_str(), itn->second.c_str());
		} else {
			ROS_ERROR("[BciState] Cannot call service '%s'='%s'", 
							itn->first.c_str(), itn->second.c_str());
		}
	}
	

}


	}
}


#endif
