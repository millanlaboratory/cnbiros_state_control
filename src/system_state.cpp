#include <ros/ros.h>
#include "cnbiros_state_control/SystemState.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "system_state");
	
	cnbiros::control::SystemState t;

	
	ros::spin();

	return 0;

}
