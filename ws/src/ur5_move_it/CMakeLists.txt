cmake_minimum_required(VERSION 3.1.3)
project(ur5_moveit_config)

# C++17
set(CMAKE_CXX_STANDARD 17)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	roslib
	moveit_ros_move_group
	moveit_fake_controller_manager
	moveit_kinematics
	moveit_planners_ompl
	moveit_ros_visualization
	moveit_setup_assistant
	joint_state_publisher
	robot_state_publisher
	tf2_ros
	xacro
	moveit_ros_planning_interface
	gazebo_ros
	rviz
	ur5_description
)

# Finding Eigen is somewhat complicated because of our need to support Ubuntu
# all the way back to saucy.  First we look for the Eigen3 cmake module
# provided by the libeigen3-dev on newer Ubuntu.  If that fails, then we
# fall-back to the version provided by cmake_modules, which is a stand-in.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
	find_package(cmake_modules REQUIRED)
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

# Note that eigen 3.2 (on Ubuntu Wily) only provides EIGEN3_INCLUDE_DIR,
# not EIGEN3_INCLUDE_DIRS, so we have to set the latter from the former.
if(NOT EIGEN3_INCLUDE_DIRS)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		moveit_utils
	CATKIN_DEPENDS
		roscpp
		roslib
		moveit_ros_move_group
		moveit_fake_controller_manager
		moveit_kinematics
		moveit_planners_ompl
		moveit_ros_visualization
		moveit_setup_assistant
		joint_state_publisher
		robot_state_publisher
		tf2_ros
		xacro
		moveit_ros_planning_interface
		gazebo_ros
		rviz
		ur5_description
	DEPENDS
		EIGEN3
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

# libraries

## dynamics ur5
add_library(moveit_utils src/lib_moveit_utils.cpp)
target_link_libraries(moveit_utils ${catkin_LIBRARIES})

install(
	TARGETS moveit_utils
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install all project namespaced headers (package/include)
install(
	DIRECTORY include/moveit_utils/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# moveit configs
install(
	DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN "setup_assistant.launch" EXCLUDE
)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

################################################################################################

# nodes

# gazebo2rviz
add_executable(gazebo2rviz src/gazebo2rviz.cpp)
target_link_libraries(gazebo2rviz ${catkin_LIBRARIES} moveit_utils)