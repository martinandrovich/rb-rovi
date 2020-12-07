cmake_minimum_required(VERSION 2.8.3)
project(rovi_pose_estimator)

# C++17
set(CMAKE_CXX_STANDARD 17)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	roslib
	std_msgs
	sensor_msgs
	image_transport
	cv_bridge
	pcl_ros
	rovi_utils
	rovi_gazebo
)

find_package(OpenCV 4)
find_package(PCL 1.8)

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
		lib_rovi_pose_estimator
	CATKIN_DEPENDS
		roscpp
		roslib
		std_msgs
		sensor_msgs
		image_transport
		cv_bridge
		pcl_ros
		rovi_utils
		rovi_gazebo
	DEPENDS
		EIGEN3
		OpenCV
		PCL
)

# include dirs
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

################################################################################################

# libraries

# lib_rovi_gazebo (rovi_gazebo/rovi_gazebo.h)
add_library(lib_rovi_pose_estimator src/lib_rovi_pose_estimator.cpp)
target_link_libraries(lib_rovi_pose_estimator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# # 
install(
	TARGETS lib_rovi_pose_estimator
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install all project namespaced headers (package/include)
install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##################################Danielsstuff################################################### 

# find_program(CCACHE_FOUND ccache)
# if(CCACHE_FOUND)
#     set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
#     set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
# endif(CCACHE_FOUND) 
# set (CMAKE_CXX_FLAGS "-DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS")

# install lib_rovi_pose_est_M1
# add_library(lib_rovi_pose_est_M1 src/lib_rovi_pose_est_M1.cpp)
# target_link_libraries(lib_rovi_pose_est_M1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# # 
# install(
# 	TARGETS lib_rovi_pose_est_M1
# 	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# # install all project namespaced headers (package/include)
# install(
# 	DIRECTORY include/${PROJECT_NAME}/
# 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# add_library(lib_rovi_pose_est_M3 src/lib_rovi_pose_est_M3.cpp)
# target_link_libraries(lib_rovi_pose_est_M3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# # 
# install(
# 	TARGETS lib_rovi_pose_est_M3
# 	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# ## install all project namespaced headers (package/include)
# install(
# 	DIRECTORY include/${PROJECT_NAME}/
# 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

################################################################################################

# nodes

## example executable (rosrun)
## add_executable(pose_example src/pose_example.cpp)
## target_link_libraries(pose_example ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} lib_rovi_pose_estimator )
## target_include_directories(example PUBLIC include/rovi_pose_estimator)pose_example

#add_executable(pose_M1 src/pose_M1.cpp)
#target_link_libraries(pose_M1 lib_rovi_pose_est_M1 )

#add_executable(pose_M3 src/pose_M3.cpp)
#target_link_libraries(pose_M3 lib_rovi_pose_est_M3)

add_executable(pose_M4 src/pose_example.cpp)
target_link_libraries(pose_M4 lib_rovi_pose_estimator)
