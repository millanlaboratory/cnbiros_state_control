set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_state_control)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation
	cnbiros_bci)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
	FILES
	SystemStateMsg.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS 
  include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  message_runtime
)

add_library(cnbiros_state_control
			src/SystemState.cpp
			src/SystemStateConverter.cpp
			src/SystemManager.cpp
			src/BciHandler.cpp
			src/JoystickHandler.cpp
			src/NavigationHandler.cpp)
add_dependencies(cnbiros_state_control cnbiros_state_control_generate_messages_cpp)
target_link_libraries(cnbiros_state_control ${catkin_LIBRARIES})

add_executable(test_manager src/test_manager.cpp)
target_link_libraries(test_manager cnbiros_state_control ${catkin_LIBRARIES})

#add_executable(system_state src/system_state.cpp)
#target_link_libraries(system_state cnbiros_state_control ${catkin_LIBRARIES})
#
add_executable(bci_handler src/bci_handler.cpp)
target_link_libraries(bci_handler cnbiros_state_control ${catkin_LIBRARIES})

add_executable(joystick_handler src/joystick_handler.cpp)
target_link_libraries(joystick_handler cnbiros_state_control ${catkin_LIBRARIES})

add_executable(navigation_handler src/navigation_handler.cpp)
target_link_libraries(navigation_handler cnbiros_state_control ${catkin_LIBRARIES})

