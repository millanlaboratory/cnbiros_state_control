#ifndef CNBIROS_STATE_CONTROL_SYSTEMMANAGER_CPP
#define CNBIROS_STATE_CONTROL_SYSTEMMANAGER_CPP

#include "cnbiros_state_control/SystemManager.hpp"

namespace cnbiros {
	namespace control {

SystemManager::SystemManager(void) : private_nh_("~") {

	this->configure();
	
	// Add publisher
	this->pub_ = this->nh_.advertise<cnbiros_state_control::SystemStateMsg>(this->ptopic_, 1);
}

SystemManager::~SystemManager(void) {}

bool SystemManager::configure(void) {

	std::string initial_state;
	this->ptopic_ = "control/system_state";

	this->configure_states();
	this->configure_transitions();

	this->state_.DumpStates();
	this->state_.DumpTransitions();

	// Get initial state
	this->private_nh_.param<std::string>("initial_state", initial_state, "Idle");
	this->state_.Set(initial_state);

	return true;
}


bool SystemManager::on_received_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res, std::string caller) {
	
	cnbiros_state_control::SystemStateMsg	msg;

	if(this->state_.GetLabel().compare(caller) == 0)
		return true;

	if(this->state_.Change(caller) == false) {
		ROS_WARN("[SystemManager] - Transition %s=>%s not allowed", this->state_.GetLabel().c_str(), caller.c_str());
		return true;
	}

	if(SystemStateConverter::ToMessage(this->state_, msg))
		this->pub_.publish(msg);

	return true;
}

bool SystemManager::add_service(std::string id, std::string name) {

	auto ret = this->names_map_.insert(std::pair<std::string, std::string>(id, name));

	if(ret.second == true) {
		this->services_map_[id] = this->nh_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(name, boost::bind(&SystemManager::on_received_service, this, _1, _2, id));
	}

	return ret.second;
}

bool SystemManager::configure_states(void) {

	XmlRpc::XmlRpcValue dict;
	int id;
	std::string label, service;
	std::vector<int> dict_ids;
	std::vector<std::string> dict_labels;
	std::vector<std::string> dict_services;


	// Getting dictionary for the states 
	if(this->private_nh_.getParam("states", dict) == false) {
		ROS_ERROR("Cannot retrieve required states dictionary.");
		return false;
	}
	
	try {
		for(auto i = 0; i<dict.size(); ++i) {
			if(dict[i].hasMember("id") == false || dict[i].hasMember("service") == false ||
				dict[i].hasMember("label") == false) {
				ROS_ERROR("State dictionary must have fields 'id', 'label' and 'service'");
				return false;
			}
			id		= static_cast<int>(dict[i]["id"]);
			label	= static_cast<std::string>(dict[i]["label"]);
			service = static_cast<std::string>(dict[i]["service"]);

			// Store fields
			dict_ids.push_back(id);
			dict_labels.push_back(label);
			dict_services.push_back(service);
		}
	} catch (XmlRpc::XmlRpcException& ex) {
		ROS_ERROR("Wrong format for services dictionary.");
		return false;
	}


	// Fill state dictionary and create services
	for(auto i=0; i<dict_ids.size(); i++) {
		
		if(this->state_.Add(dict_ids.at(i), dict_labels.at(i)) == false)
			ROS_WARN("[SystemManager] Cannot add state: %s", dict_labels.at(i).c_str());

		if(this->add_service(dict_labels.at(i), dict_services.at(i)) == false)
			ROS_WARN("[SystemManager] Cannot add service: %s", dict_labels.at(i).c_str());
	}

	return true;
}


bool SystemManager::configure_transitions(void) {
	
	XmlRpc::XmlRpcValue dict;
	std::string from, to;
	std::vector<std::string> dict_from;
	std::vector<std::string> dict_to;


	// Getting dictionary for the states 
	if(this->private_nh_.getParam("transitions", dict) == false) {
		ROS_ERROR("Cannot retrieve required transitions dictionary.");
		return false;
	}
	
	try {
		for(auto i = 0; i<dict.size(); ++i) {
			if(dict[i].hasMember("from") == false || dict[i].hasMember("to") == false) {
				ROS_ERROR("Transitions dictionary must have fields 'from' and 'to'");
				return false;
			}
			from = static_cast<std::string>(dict[i]["from"]);
			to	 = static_cast<std::string>(dict[i]["to"]);

			// Store fields
			dict_from.push_back(from);
			dict_to.push_back(to);
		}
	} catch (XmlRpc::XmlRpcException& ex) {
		ROS_ERROR("Wrong format for transitions dictionary.");
		return false;
	}


	// Fill transitions dictionary
	for(auto i=0; i<dict_from.size(); i++) {
		if(this->state_.AddTransition(dict_from.at(i), dict_to.at(i)) == false)
			ROS_WARN("[SystemManager] Cannot add transition: %s->%s", dict_from.at(i).c_str(), dict_to.at(i).c_str());
	}

	return true;

}

	}
}

#endif
