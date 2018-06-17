#include <ros/ros.h>
#include "cnbiros_state_control/JoystickHandler.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "joystick_handler");
	
	cnbiros::control::JoystickHandler t;

	
	ros::spin();

	return 0;

}
