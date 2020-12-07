#include "rovi_gazebo/rovi_gazebo.h"

#include <iostream>
#include <thread>
#include <mutex>
#include <regex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <sensor_msgs/CameraInfo.h>

#include <ur5_dynamics/ur5_dynamics.h>
#include <rovi_utils/rovi_utils.h>

Eigen::Isometry3d
rovi_gazebo::w_T_b()
{
	Eigen::Isometry3d w_T_b;
	tf::poseMsgToEigen(rovi_gazebo::get_current_base_pose(), w_T_b);

	return w_T_b;
}

void
rovi_gazebo::set_simulation(bool state)
{
	static auto nh = new ros::NodeHandle("~/gazebo_simulation_controller");
	static auto client_gazebo_sim = nh->serviceClient<std_srvs::Empty>((state) ? "/gazebo/unpause_physics" : "/gazebo/pause_physics");
	std_srvs::Empty srv;
	client_gazebo_sim.call(srv);
}

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


sensor_msgs::CameraInfo
rovi_gazebo::get_camera_info()
{
	return *ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/rbrovi/camera/camera_info");
}

std::unordered_map<std::string, cv::Mat>
rovi_gazebo::get_stereo_camera_imgs()
{
	// construct listener threads (once)
	static std::mutex mutex_img_l, mutex_img_r;
	static sensor_msgs::Image img_l, img_r;
	static auto thread_l = rovi_utils::create_async_listener("/rbrovi/camera_stereo/left/image_raw",  img_l, mutex_img_l);
	static auto thread_r = rovi_utils::create_async_listener("/rbrovi/camera_stereo/right/image_raw", img_r, mutex_img_r);

	// map of images
	static std::unordered_map<std::string, cv::Mat> imgs
	{
		{ "left",  cv::Mat() },
		{ "right", cv::Mat() },
	};

	std::lock_guard<std::mutex> lock_l(mutex_img_l);
	std::lock_guard<std::mutex> lock_r(mutex_img_r);

	imgs["left"]  = cv_bridge::toCvCopy(boost::make_shared<const sensor_msgs::Image>(img_l), "bgr8")->image;
	imgs["right"] = cv_bridge::toCvCopy(boost::make_shared<const sensor_msgs::Image>(img_r), "bgr8")->image;

	return imgs;
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

			// remove any digits from string (e.g. bottle3 -> bottle)
			auto model_name = obj;
			model_name.erase(std::remove_if(model_name.begin(), model_name.end(), &isdigit), model_name.end());

			// construct and add colision object
			ROS_INFO_STREAM("Adding object: " << obj << " (" << model_name << ") at " << "[" << pos.x << ", " << pos.y << ", " << pos.z << "]");
			collision_objects.emplace_back(rovi_utils::make_mesh_cobj(obj, planning_frame, { pos.x, pos.y, pos.z }, { ori.w, ori.x, ori.y, ori.z }));
		}
		else
			;// ROS_WARN_STREAM("Excluding object: " << obj);
	}

	return collision_objects;
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

void
rovi_gazebo::spawn_model(const std::string& model, const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy)
{
	if (not std::regex_match(name, std::regex("^" + model + "[0-9]+$")))
	{
		ROS_ERROR_STREAM("spawn_model(): object name must follow the pattern '<model>n' where n is some number (e.g. bottle1)");
		return;
	}
	
	// https://pastebin.com/UTWJSScZ (CANNOT DEFINE DATABASE MODEL)
	// static auto nh = new ros::NodeHandle("~/gazebo_model_spawner");
	// static ros::ServiceClient client_gazebo_spawn = nh->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
	// gazebo_msgs::SpawnModel srv;

	// define command
	auto cmd = std::string("rosrun gazebo_ros spawn_model") +
		" -database " + model + " -sdf" +
		" -model " + name +
		" -x " + std::to_string(pos[0]) +
		" -y " + std::to_string(pos[1]) +
		" -z " + std::to_string(pos[2]) +
		" -R " + std::to_string(rpy[0]) +
		" -P " + std::to_string(rpy[1]) +
		" -Y " + std::to_string(rpy[2]);
	
	// execute command
	system(cmd.c_str());
}

void
rovi_gazebo::move_model(const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy)
{
	static auto nh = new ros::NodeHandle("~/gazebo_model_setter");
	static auto client = nh->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	gazebo_msgs::SetModelState srv;
	
	geometry_msgs::Pose pose;
	if (rpy[0] == INFINITY)
	{
		// keep current orientation
		pose = get_model_pose(name);
		pose.position.x = pos[0];
		pose.position.y = pos[1];
		pose.position.z = pos[2];
	}
	else
		pose = rovi_utils::make_pose(pos, rpy);
	
	srv.request.model_state.model_name = name;
	srv.request.model_state.reference_frame = "world";
	srv.request.model_state.pose = pose;
	auto success = client.call(srv);
}

void
rovi_gazebo::remove_model(const std::string& name)
{
	static auto nh = new ros::NodeHandle("~/gazebo_model_deleter");
	static auto client = nh->serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
	gazebo_msgs::DeleteModel srv;
	srv.request.model_name = name;
	auto success = client.call(srv);
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
	auto pose_l6      = link_states.pose[idx_link6];

	// controller uses pose given in robot base frame; offset from world
	pose_l6.position.x -= (in_world_frame) ? 0 : offset.position.x;
	pose_l6.position.y -= (in_world_frame) ? 0 : offset.position.y;
	pose_l6.position.z -= (in_world_frame) ? 0 : offset.position.z;

	return pose_l6;
}

geometry_msgs::Pose
rovi_gazebo::get_current_ee_pose()
{
	const auto pose_l6 = get_current_link6_pose(false);
	Eigen::Isometry3d b_T_l6, b_T_ee;

	tf::poseMsgToEigen(pose_l6, b_T_l6);

	b_T_ee = b_T_l6 * ur5_dynamics::l6_T_ee;

	// return as pose
	geometry_msgs::Pose pose_ee;
	tf::poseEigenToMsg(b_T_ee, pose_ee);
	return pose_ee;
}

geometry_msgs::Pose
rovi_gazebo::get_current_tcp_pose()
{
	const auto pose_l6 = get_current_link6_pose(false);
	Eigen::Isometry3d b_T_l6, b_T_tcp;

	tf::poseMsgToEigen(pose_l6, b_T_l6);

	b_T_tcp = b_T_l6 * ur5_dynamics::l6_T_ee * ur5_dynamics::ee_T_tcp;

	// return as pose
	geometry_msgs::Pose pose_tcp;
	tf::poseEigenToMsg(b_T_tcp, pose_tcp);
	return pose_tcp;
}

geometry_msgs::Pose
rovi_gazebo::get_ee_given_pos(const geometry_msgs::Pose& pose_obj, const Eigen::Isometry3d& offset)
{

	// w_T_tcp = w_T_b * b_T_tcp -> b_T_tcp = inv(w_T_b) * w_T_tcp, where w_T_tcp = w_T_obj
	// b_T_tcp = b_T_ee * ee_T_tcp -> b_T_ee = b_T_tcp * inv(ee_T_tcp)

	Eigen::Isometry3d b_T_tcp, w_T_obj, obj_T_offset, b_T_ee;
	tf::poseMsgToEigen(pose_obj, w_T_obj); // w_T_tcp = w_T_obj

	// offset (in world/object frame)
	obj_T_offset = offset;

	b_T_tcp = w_T_b().inverse() * w_T_obj * obj_T_offset;
	b_T_ee = b_T_tcp * ur5_dynamics::ee_T_tcp.inverse();

	// return as pose
	geometry_msgs::Pose pose_ee;
	tf::poseEigenToMsg(b_T_ee, pose_ee);
	return pose_ee;
}

geometry_msgs::Pose
rovi_gazebo::get_tcp_given_pos(const geometry_msgs::Pose& pose_obj, const Eigen::Isometry3d& offset)
{
	// given object pose (in world frame), compute the desired TCP pose in robot frame
	// w_T_tcp = w_T_b * b_T_tcp -> b_T_tcp = inv(w_T_b) * w_T_tcp, where w_T_tcp = w_T_obj

	Eigen::Isometry3d b_T_tcp, w_T_obj, obj_T_offset;
	tf::poseMsgToEigen(pose_obj, w_T_obj); // w_T_tcp = w_T_obj

	// offset (in world/object frame)
	obj_T_offset = offset;

	b_T_tcp = w_T_b().inverse() * w_T_obj * obj_T_offset;

	// return as pose
	geometry_msgs::Pose pose_tcp;
	tf::poseEigenToMsg(b_T_tcp, pose_tcp);
	return pose_tcp;
}