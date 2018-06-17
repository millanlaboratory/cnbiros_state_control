#ifndef CNBIROS_STATE_CONTROL_SYSTEMSTATE_HPP
#define CNBIROS_STATE_CONTROL_SYSTEMSTATE_HPP


// System includes
#include <string>
#include <unordered_map>
#include <map>

namespace cnbiros {
	namespace control {


class SystemState {


	public:
		SystemState(void);
		virtual ~SystemState(void);

		bool Add(int id, std::string label);
		bool Set(int id);
		bool Set(std::string label);
		bool Is(int id);
		bool Is(std::string label);
		unsigned int GetSize(void);
		
		int GetId(void);
		std::string GetLabel(void);
		
		int GetPrevId(void);
		std::string GetPrevLabel(void);

		bool AddTransition(std::string from, std::string to);
		bool Change(std::string next_state);

		void DumpStates(void);
		void DumpTransitions(void);

	private:

		int			id_;
		std::string label_;
		
		int			prev_id_;
		std::string	prev_label_;
		
		std::unordered_map<int, std::string> dictionary_;
		std::multimap<std::string, std::string> transitions_;

};

	}
}




#endif
