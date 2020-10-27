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
	sensor_msgs
	cv_bridge
	image_transport
	message_generation
	std_msgs
	)
find_package(
	OpenCV REQUIRED
	)
find_package(
	PCL 1.2 REQUIRED
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


#add service files

add_service_files(
	DIRECTORY srv
	FILES
	AddTwoInts.srv
)

#add message_files
add_message_files(
	DIRECTORY
	msg
	FILES
	Num.msg
)

#call generate function
generate_messages(
	DEPENDENCIES
	std_msgs
)



# declare a catkin package
catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		lib_rovi_pose_estimator
	CATKIN_DEPENDS
		roscpp
		roslib
		sensor_msgs
		cv_bridge
		image_transport
		message_runtime
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

## lib_rovi_gazebo (rovi_gazebo/rovi_gazebo.h)
add_library(lib_rovi_pose_estimator src/lib_rovi_pose_estimator.cpp)
target_link_libraries(lib_rovi_pose_estimator ${catkin_LIBRARIES} ${PCL_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
## 
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

################################################################################################

# nodes

## example executable (rosrun)
add_executable(pose_example src/pose_example.cpp)
target_link_libraries(pose_example ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} lib_rovi_pose_estimator )
add_dependencies(pose_example rovi_pose_estimator_generate_messages_cpp)
##target_include_directories(example PUBLIC include/rovi_pose_estimator)pose_example

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(listener rovi_pose_estimator_generate_messages_cpp)


add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server rovi_pose_estimator_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client rovi_pose_estimator_gencpp)