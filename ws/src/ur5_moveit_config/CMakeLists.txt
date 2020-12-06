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
	xacro
	gazebo_ros
	rviz
	tf2_ros
	pluginlib
	joint_state_publisher
	robot_state_publisher
	moveit_core
	moveit_ros_move_group
	moveit_fake_controller_manager
	moveit_kinematics
	moveit_planners_ompl
	moveit_visual_tools
	moveit_ros_visualization
	moveit_setup_assistant
	moveit_ros_planning_interface
	ur5_description
)

find_package(Boost REQUIRED)

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
	LIBRARIES
	CATKIN_DEPENDS
		roscpp
		roslib
		xacro
		gazebo_ros
		rviz
		tf2_ros
		pluginlib
		joint_state_publisher
		robot_state_publisher
		moveit_core
		moveit_ros_move_group
		moveit_fake_controller_manager
		moveit_kinematics
		moveit_planners_ompl
		moveit_visual_tools
		moveit_ros_visualization
		moveit_setup_assistant
		moveit_ros_planning_interface
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

# moveit configs
install(
	DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN "setup_assistant.launch" EXCLUDE
)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

################################################################################################

# nodes

# rrt_test
#add_executable(rrt_test src/rrt_test.cpp)
#target_link_libraries(rrt_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# ur5_reachability
#add_executable(ur5_reachability src/ur5_reachability.cpp)
#target_link_libraries(ur5_reachability ${catkin_LIBRARIES})

# plan_test
#add_executable(plan_test src/plan_test.cpp)
#target_link_libraries(plan_test ${catkin_LIBRARIES})

# plan_test
#add_executable(plan_scene src/plan_scene.cpp)
#target_link_libraries(plan_scene ${catkin_LIBRARIES})
