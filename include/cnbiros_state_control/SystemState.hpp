#ifndef CNBIROS_STATE_CONTROL_SYSTEMSTATE_HPP
#define CNBIROS_STATE_CONTROL_SYSTEMSTATE_HPP


// Ros includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>

namespace cnbiros {
	namespace control {

enum class State {Idle, Joystick, Navigation, Stopped, Running};

class SystemState {


	public:
		SystemState(void);
		virtual ~SystemState(void);

		bool configure(void);

		bool IsState(State state);
		void Run(void);

	private:
		bool on_joystick_control(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
		bool on_navigation_control(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
		bool on_navigation_stop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
		bool on_navigation_start(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);


		void request_navigation_stop(void);
		void request_navigation_start(void);
		void request_navigation_enable(void);
		void request_navigation_disable(void);
		void request_joystick_enable(void);
		void request_joystick_disable(void);

	private:
		ros::NodeHandle	nh_;
		ros::NodeHandle	p_nh_;

		ros::ServiceServer	srv_joystick_control_;
		ros::ServiceServer	srv_navigation_control_;
		ros::ServiceServer	srv_navigation_start_;
		ros::ServiceServer	srv_navigation_stop_;

		ros::ServiceClient	cli_joystick_enable_;
		ros::ServiceClient	cli_joystick_disable_;
		ros::ServiceClient	cli_navigation_enable_;
		ros::ServiceClient	cli_navigation_disable_;
		ros::ServiceClient	cli_navigation_start_;
		ros::ServiceClient	cli_navigation_stop_;

		State	state_;

};


	}
}




#endif
