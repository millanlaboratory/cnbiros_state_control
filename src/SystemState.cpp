#ifndef CNBIROS_STATE_CONTROL_SYSTEMSTATE_CPP
#define CNBIROS_STATE_CONTROL_SYSTEMSTATE_CPP

#include "cnbiros_state_control/SystemState.hpp"

namespace cnbiros {
	namespace control {

SystemState::SystemState(void)  {
	this->id_ = -1;
	this->prev_id_ = -1;

	this->label_ = "Unset";
	this->prev_label_ = "Unset";

}

SystemState::~SystemState(void) {}

bool SystemState::Is(int id) {
	return this->id_ == id ? true : false;
}

bool SystemState::Is(std::string label) {

	bool result = false;
	if(this->label_.compare(label) == 0)
		result = true;
	return result;
}


bool SystemState::Add(int id, std::string label) {

	bool is_duplicated = false;
	bool result = false;

	// Search for entry with this id in the dictionnary
	auto it = this->dictionary_.find(id);

	// Check if already exists
	if(it != this->dictionary_.end() )
		return false;

	// If does not exists, iterate on labels

	// Check for uniquness of the label
	for(auto it=this->dictionary_.begin(); it != this->dictionary_.end(); ++it) {
		if(it->second.compare(label) == 0) { // if found
			is_duplicated = true;
			break;
		}
	}

	if (is_duplicated == false) {
		auto ret = this->dictionary_.insert(std::pair<int, std::string>(id, label));
		result = ret.second;
	}

	return result;
}

bool SystemState::AddTransition(std::string from, std::string to) {

	auto ret = this->transitions_.insert(std::pair<std::string, std::string>(from, to));

	return true;

}

bool SystemState::Change(std::string next_state) {

	bool allowed = false;
	bool result  = false;
	auto ret = this->transitions_.equal_range(this->label_);

	for(auto it = ret.first; it !=ret.second; ++it) {
		
		if(it->second.compare(next_state) == 0) {
			allowed = true;
			break;
		}
	}

	if( allowed == true ) {
		this->Set(next_state);
		result = true;
	}

	return result;
}

bool SystemState::Set(int id) {

	bool ret = false;
	auto it = this->dictionary_.find(id);

	if( it != this->dictionary_.end() ) {
		this->prev_id_		= this->id_;
		this->prev_label_	= this->label_;
		this->id_			= it->first;
		this->label_		= it->second;
		ret = true;
	}
	return ret;
}

bool SystemState::Set(std::string label) {

	bool found  = false;
	bool result = false;
	int id;

	for(auto it=this->dictionary_.begin(); it != this->dictionary_.end(); ++it) {
		if(it->second.compare(label) == 0) { // if found
			found = true;
			id = it->first;
			break;
		}
	}

	if(found == true) {
		result = this->Set(id);
	}

	return result;
}

unsigned int SystemState::GetSize(void) {
	return this->dictionary_.size();
}

int SystemState::GetId(void) {
	return this->id_;
}

std::string SystemState::GetLabel(void) {
	return this->label_;
}

int SystemState::GetPrevId(void) {
	return this->prev_id_;
}

std::string SystemState::GetPrevLabel(void) {
	return this->prev_label_;
}

void SystemState::DumpStates(void) {

	printf("[SystemState] States dictionary:\n");
	for(auto it=this->dictionary_.begin(); it != this->dictionary_.end(); ++it) 
		printf("		- %s\n", it->second.c_str());
}

void SystemState::DumpTransitions(void) {

	printf("[SystemState] Transitions dictionary:\n");
	for(auto it=this->transitions_.begin(); it != this->transitions_.end(); ++it) 
		printf("		- %s => %s\n", it->first.c_str(), it->second.c_str());
}

	}
}


#endif
