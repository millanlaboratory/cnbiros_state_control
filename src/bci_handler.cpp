#include <ros/ros.h>
#include "cnbiros_state_control/BciHandler.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "bci_handler");
	
	cnbiros::control::BciHandler t;

	
	ros::spin();

	return 0;

}
