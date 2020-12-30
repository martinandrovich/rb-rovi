cmake_minimum_required(VERSION 2.8.3)
project(rovi_system)

# C++17
set(CMAKE_CXX_STANDARD 17)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	roslib
	rovi_utils
	rovi_gazebo
	rovi_planner
	# rovi_pose_estimator
)

# find other libraries
find_package(OpenCV 4)
# find_package(PCL 1.8)

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
		rovi_utils
		rovi_gazebo
		rovi_planner
		# rovi_pose_estimator
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

# nodes

add_executable(interpolation_kdl src/interpolation_kdl.cpp)
target_link_libraries(interpolation_kdl ${catkin_LIBRARIES})

add_executable(reachability src/reachability.cpp)
target_link_libraries(reachability ${catkin_LIBRARIES})

add_executable(planning_kdl src/planning_kdl.cpp)
target_link_libraries(planning_kdl ${catkin_LIBRARIES})

add_executable(planning_rrt src/planning_rrt.cpp)
target_link_libraries(planning_rrt ${catkin_LIBRARIES})