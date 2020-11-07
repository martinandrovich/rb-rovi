cmake_minimum_required(VERSION 3.1.3)
project(ur5_move_it)

# C++17
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
            rviz
            moveit_ros_planning_interface
            gazebo_ros
            )

catkin_package(
              INCLUDE_DIRS
                include
              LIBRARIES
                
              CATKIN_DEPENDS
                rviz
                moveit_ros_planning_interface
                gazebo_ros
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# node

add_executable(gazebo2rviz src/gazebo2rviz.cpp)
target_link_libraries(gazebo2rviz ${catkin_LIBRARIES})