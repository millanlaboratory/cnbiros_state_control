set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_state_control)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation
	cnbiros_bci)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
include_directories(include ${catkin_INCLUDE_DIRS})


catkin_package(
  INCLUDE_DIRS 
  include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
)

add_library(cnbiros_state_control
			src/SystemState.cpp
			src/BciState.cpp)
target_link_libraries(cnbiros_state_control ${catkin_LIBRARIES})

add_executable(system_state src/system_state.cpp)
target_link_libraries(system_state cnbiros_state_control ${catkin_LIBRARIES})

add_executable(bci_state src/bci_state.cpp)
target_link_libraries(bci_state cnbiros_state_control ${catkin_LIBRARIES})

