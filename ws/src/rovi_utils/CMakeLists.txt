cmake_minimum_required(VERSION 2.8.3)
project(rovi_utils)

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
	tf
	geometry_msgs
	moveit_msgs
	geometric_shapes
	moveit_ros_planning_interface
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
		rovi_utils
	CATKIN_DEPENDS
		roscpp
		roslib
		xacro
		gazebo_ros
		tf
		geometry_msgs
		moveit_msgs
		geometric_shapes
		moveit_ros_planning_interface
	DEPENDS
		EIGEN3
)

# include dirs
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

################################################################################################

# libraries

## rovi_utils (rovi_utils/rovi_utils.h)
add_library(rovi_utils src/lib_rovi_utils.cpp)
target_link_libraries(rovi_utils ${catkin_LIBRARIES})

install(
	TARGETS
		rovi_utils
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install all project namespaced headers (package/include)
install(
	DIRECTORY
		include/rovi_utils
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)