cmake_minimum_required(VERSION 2.8.3)

project(ur5_controllers)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED
    
    robot_state_publisher
    joint_state_controller
    joint_state_publisher
    controller_interface
    controller_manager
    position_controllers
    effort_controllers
    std_msgs
    gazebo_ros
    urdf 
    kdl_parser

)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

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
