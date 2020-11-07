#!/bin/bash

# CONTROLLERS="['ur5_joint_position_pd_gravity_controller', 'ur5_joint_state_controller', 'ur5_joint_state_controller']"

# stop controllers
#rosservice call /controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['ur5_joint_position_pd_gravity_controller','ur5_joint_state_controller', 'ur5_joint_state_controller'], strictness: 2}"

# pause simulation
rosservice call /gazebo/pause_physics

# delete model
rosservice call /gazebo/delete_model '{model_name: ur5}'

# spawn model at new pos
rosrun gazebo_ros spawn_model -param robot_description -urdf -model ur5 -x 0.2 -y 0.2 -z 0.75 -J ur5_joint1 0 -J ur5_joint2 -1.57 -J ur5_joint3 0 -J ur5_joint4 0 -J ur5_joint5 0 -J ur5_joint6 0 -J wsg_base_mount_right_joint 0 -J wsg_base_mount_left_joint 0

# spawn + start controllers
#rosservice call /controller_manager/switch_controller "{start_controllers: ['ur5_joint_position_pd_gravity_controller','ur5_joint_state_controller', 'ur5_joint_state_controller'], stop_controllers: [], strictness: 2}"
rosrun controller_manager spawner ur5_joint_state_controller ur5_joint_position_pd_gravity_controller &

# reset simulation time and start
rosservice call /gazebo/reset_simulation
rosservice call /gazebo/unpause_physics