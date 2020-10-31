cmake_minimum_required(VERSION 2.8.3)
project(rovi_planner)

# C++17
set(CMAKE_CXX_STANDARD 17)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	roslib
	# orocos_kdl -> melodic
	# liborocos-kdl-dev -> noetic
	kdl_parser
	tf_conversions
	eigen_conversions
	kdl_conversions
	ur5_dynamics
	ur5_controllers
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
	CATKIN_DEPENDS
		roscpp
		roslib
		# orocos_kdl -> melodic
		# liborocos-kdl-dev -> noetic
		kdl_parser
		tf_conversions
		eigen_conversions
		kdl_conversions
		ur5_dynamics
		ur5_controllers
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

## ...
add_library(rovi_planner src/librovi_planner.cpp)
target_link_libraries(rovi_planner ${catkin_LIBRARIES})

install(
	TARGETS rovi_planner
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install all project namespaced headers (package/include)
install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################################

# nodes

add_executable(traj_test src/traj_test.cpp)
target_link_libraries(traj_test ${catkin_LIBRARIES} rovi_planner)

# ...