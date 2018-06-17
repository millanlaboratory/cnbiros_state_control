#include <ros/ros.h>
#include "cnbiros_state_control/NavigationHandler.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "navigation_handler");
	
	cnbiros::control::NavigationHandler t;

	
	ros::spin();

	return 0;

}
