#include <ros/ros.h>
#include "cnbiros_state_control/BciState.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "bci_state");
	
	cnbiros::control::BciState t;

	
	ros::spin();

	return 0;

}
