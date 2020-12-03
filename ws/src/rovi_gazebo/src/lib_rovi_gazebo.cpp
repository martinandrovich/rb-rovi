#include "rovi_gazebo/rovi_gazebo.h"

#include <rovi_utils/rovi_utils.h>

#include <ros/ros.h>
#include <iostream>
#include <thread>

#include <std_msgs/Int32.h>

void
rovi_gazebo::set_projector(bool state)
{
	
	// create atomic bool and async publisher
	static std::atomic<bool> state_;
	static auto t = std::thread([&]()
	{
		auto nh = new ros::NodeHandle("~/projector_publisher");
		auto pub_projector = nh->advertise<std_msgs::Int32>("/projector_controller/projector", 1);		
		std_msgs::Int32 msg;
		ros::Rate lp(100); // Hz
		while (ros::ok())
		{
			msg.data = (state_) ? 1 : 0;
			pub_projector.publish(msg);
			lp.sleep();
		}
	});
	
	// set async state
	state_ = state;
}

gazebo_msgs::LinkStates
rovi_gazebo::get_link_states()
{
	// construct listener thread (once)
	static std::mutex mtx_link_states;
	static gazebo_msgs::LinkStates link_states;
	static auto t = rovi_utils::create_async_listener("/gazebo/link_states", link_states, mtx_link_states);

	// return mutexed link states
	std::lock_guard<std::mutex> lock(mtx_link_states);
	return link_states;
}

gazebo_msgs::ModelStates
rovi_gazebo::get_model_states()
{
	// construct listener thread (once)
	static std::mutex mtx_model_states;
	static gazebo_msgs::ModelStates model_states;
	static auto t = rovi_utils::create_async_listener("/gazebo/model_states", model_states, mtx_model_states);

	// return mutexed model states
	std::lock_guard<std::mutex> lock(mtx_model_states);
	return model_states;
}

sensor_msgs::JointState
rovi_gazebo::get_joint_states()
{

	// construct listener thread (once)
	static std::mutex mtx_joint_states;
	static sensor_msgs::JointState joint_states;
	static auto t = rovi_utils::create_async_listener("/joint_states", joint_states, mtx_joint_states);

	// return mutexed joint states
	std::lock_guard<std::mutex> lock(mtx_joint_states);
	return joint_states;
}

geometry_msgs::Pose
rovi_gazebo::get_model_pose(const std::string& name)
{
	const auto model_states = get_model_states();
	const auto& model_names = model_states.name;

	const auto obj = std::find(model_names.begin(), model_names.end(), name);

	if (obj == model_names.end())
		ROS_ERROR_STREAM("Could not locate " << name << " model in Gazebo's model states.");

	size_t idx_base  = std::distance(model_names.begin(), obj);

	return rovi_utils::make_pose(
	{
		model_states.pose[idx_base].position.x,
		model_states.pose[idx_base].position.y,
		model_states.pose[idx_base].position.z,
	},
	Eigen::Quaternion<double>(
	{
		model_states.pose[idx_base].orientation.w,
		model_states.pose[idx_base].orientation.x,
		model_states.pose[idx_base].orientation.y,
		model_states.pose[idx_base].orientation.z
	}));
}

sensor_msgs::JointState
rovi_gazebo::get_current_robot_state()
{
	// get joint states
	auto joint_states = rovi_gazebo::get_joint_states();

	// filter out joint values (only keep first six joints)
	joint_states.name.resize(NUM_JOINTS);
	joint_states.position.resize(NUM_JOINTS);
	joint_states.velocity.resize(NUM_JOINTS);
	joint_states.effort.resize(NUM_JOINTS);

	return joint_states;
}

sensor_msgs::JointState
rovi_gazebo::get_current_gripper_state()
{
	// get joint states
	auto joint_states = rovi_gazebo::get_joint_states();

	// filter out joint values (remove first six joints)
	joint_states.name.erase(joint_states.name.begin(), joint_states.name.begin() + NUM_JOINTS);
	joint_states.position.erase(joint_states.position.begin(), joint_states.position.begin() + NUM_JOINTS);
	joint_states.velocity.erase(joint_states.velocity.begin(), joint_states.velocity.begin() + NUM_JOINTS);
	joint_states.effort.erase(joint_states.effort.begin(), joint_states.effort.begin() + NUM_JOINTS);

	return joint_states;
}

geometry_msgs::Pose
rovi_gazebo::get_current_base_pose()
{
	return get_model_pose("ur5");
	// const auto model_states = get_model_states();
	// const auto& model_names = model_states.name;
	// size_t idx_base  = std::distance(model_names.begin(), std::find(model_names.begin(), model_names.end(), "ur5"));

	// return rovi_utils::make_pose(
	// {
	// 	model_states.pose[idx_base].position.x,
	// 	model_states.pose[idx_base].position.y,
	// 	model_states.pose[idx_base].position.z
	// });
}

geometry_msgs::Pose
rovi_gazebo::get_current_link6_pose(bool in_world_frame)
{
	// get link states
	const auto link_states = get_link_states();

	// get current pose of base and link6 in world frame
	const auto& link_names = link_states.name;
	size_t idx_base  = std::distance(link_names.begin(), std::find(link_names.begin(), link_names.end(), "ur5::ur5_link0"));
	size_t idx_link6 = std::distance(link_names.begin(), std::find(link_names.begin(), link_names.end(), "ur5::ur5_link6"));

	const auto offset = link_states.pose[idx_base];
	auto pose_current = link_states.pose[idx_link6];

	// controller uses pose given in robot base frame; offset from world
	pose_current.position.x -= (in_world_frame) ? 0 : offset.position.x;
	pose_current.position.y -= (in_world_frame) ? 0 : offset.position.y;
	pose_current.position.z -= (in_world_frame) ? 0 : offset.position.z;

	return pose_current;
}

std::vector<moveit_msgs::CollisionObject>
rovi_gazebo::get_collision_objects(const std::string& planning_frame, const std::vector<std::string>& excludes)
{
	const auto model_states = get_model_states();
	const auto& vec_poses = model_states.pose;
	const auto& vec_obj = model_states.name;

	ROS_INFO_STREAM("Recieved gazebo_msgs::ModelStates with " << vec_obj.size() << " objects; excluding " << excludes.size() << " objects");

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	for (const auto& obj : vec_obj)
	{
		// create colllision models that are not to be excluded
		if (std::find(excludes.begin(), excludes.end(), obj) == excludes.end())
		{
			// find pose of object
			size_t i = std::distance(vec_obj.begin(), std::find(vec_obj.begin(), vec_obj.end(), obj));
			const auto& pos = vec_poses[i].position;
			const auto& ori = vec_poses[i].orientation;

			// construct and add colision object
			ROS_INFO_STREAM("Adding object: " << obj << " at " << "[" << pos.x << ", " << pos.y << ", " << pos.z << "]");
			collision_objects.emplace_back(rovi_utils::make_mesh_cobj(obj, planning_frame, { pos.x, pos.y, pos.z }, { ori.w, ori.x, ori.y, ori.z }));
		}
		else
			;// ROS_WARN_STREAM("Excluding object: " << obj);
	}

	return collision_objects;
}