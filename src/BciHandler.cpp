#ifndef CNBIROS_CONTROL_BCISTATEHANDLER_CPP
#define CNBIROS_CONTROL_BCISTATEHANDLER_CPP

#include "cnbiros_state_control/BciHandler.hpp"


namespace cnbiros {
	namespace control {

BciHandler::BciHandler(void) : p_nh_("~") {

	this->configure();

	// Initialize subscriber
	this->sub_ = this->nh_.subscribe(this->stopic_, 1, &BciHandler::on_received_tid, this);
	this->subctr_ = this->nh_.subscribe(this->ctrtopic_, 1, &BciHandler::on_received_state, this);

	// Initialize publisher
	this->pub_ = this->nh_.advertise<cnbiros_bci::TidMessage>(this->pubtopic_, 1);

	// Initialize services
	for(auto it=this->services_names_.begin(); it!=this->services_names_.end(); ++it) {
		this->services_[it->first] = this->nh_.serviceClient<std_srvs::Empty>(it->second);
		ROS_INFO("Configured services: ['%s'] => %s", it->first.c_str(), it->second.c_str());
	}

}

BciHandler::~BciHandler(void) {}

bool BciHandler::configure(void) {
	
	XmlRpc::XmlRpcValue dict;
	std::string	dict_key, dict_label;

	this->stopic_ = "/rostid_cnbi2ros";
	this->pubtopic_ = "/rostid_ros2cnbi";
	this->ctrtopic_ = "/control/system_state";

	// Getting dictionary for the events<=>angles association
	if(this->p_nh_.getParam("states", dict) == false) {
		ROS_ERROR("Cannot retrieve required states dictionary.");
		return false;
	}
	
	try {
		for(auto i = 0; i<dict.size(); ++i) {
			if(dict[i].hasMember("event") == false || dict[i].hasMember("label") == false 
					|| dict[i].hasMember("service") == false) {
				ROS_ERROR("Commands dictionary must have fields 'event', 'label', 'service'");
				return false;
			}
			dict_key = static_cast<std::string>(dict[i]["event"]);
			dict_label = static_cast<std::string>(dict[i]["label"]);
			this->services_names_[dict_key] = static_cast<std::string>(dict[i]["service"]);
			this->events_map_[dict_label] = dict_key;
		}
	} catch (XmlRpc::XmlRpcException& ex) {
		ROS_ERROR("Wrong format for services dictionary.");
		return false;
	}


	return true;
}

void BciHandler::on_received_tid(const cnbiros_bci::TidMessage& msg) {

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
		}
	}
}

void BciHandler::on_received_state(const cnbiros_state_control::SystemStateMsg& msg) {

	cnbiros_bci::TidMessage tid;
	auto it = this->events_map_.find(msg.label);

	
	if( it != this->events_map_.end() ) {

		tid.header.stamp = ros::Time::now();
		tid.pipe  = "/dev";
		std::istringstream(it->second) >> std::hex >> tid.event;
		this->pub_.publish(tid);
	}
}


	}
}


#endif
