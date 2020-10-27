cmake_minimum_required(VERSION 2.8.3)
project(ur5_controllers)

# C++17
set(CMAKE_CXX_STANDARD 17)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	roslib
	xacro
	urdf
	controller_manager
	controller_interface
	gazebo_ros
	std_msgs
	effort_controllers
	position_controllers
	joint_state_controller
	joint_state_publisher
	robot_state_publisher
	kdl_parser
	pluginlib
	ur5_dynamics
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		ur5_controllers
	CATKIN_DEPENDS
		roscpp
		roslib
		xacro
		urdf
		controller_manager
		controller_interface
		gazebo_ros
		std_msgs
		effort_controllers
		position_controllers
		joint_state_controller
		joint_state_publisher
		robot_state_publisher
		kdl_parser
		pluginlib
		ur5_dynamics
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

################################################################################################

add_library(
	ur5_kin 
	src/ur5_kin.cpp
)

target_include_directories(
	ur5_kin PUBLIC
	include
)

target_link_libraries(ur5_kin ${catkin_LIBRARIES})

add_executable(ur5_controller src/main.cpp)
target_link_libraries(ur5_controller PRIVATE ${catkin_LIBRARIES} ur5_kin)

################################################################################################

# libraries

## ...
add_library(ur5_controllers src/joint_position_pd_gravity_controller.cpp)
target_link_libraries(ur5_controllers ${catkin_LIBRARIES})

install(
	TARGETS ur5_controllers
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install all project namespaced headers (package/include)
install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## install ros_control plugin
install(
	FILES ur5_controllers_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################################

# nodes

# ...