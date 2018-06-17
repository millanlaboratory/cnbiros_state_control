#include <ros/ros.h>
#include "cnbiros_state_control/SystemManager.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_manager");
	
	cnbiros::control::SystemManager t;

	
	ros::spin();

	return 0;

}
