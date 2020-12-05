#include "rovi_utils/rovi_utils.h"

#include <thread>
#include <pthread.h>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl/trajectory_composite.hpp>

#include <moveit/planning_interface/planning_interface.h>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometric_shapes/shape_operations.h>

#include <ur5_dynamics/ur5_dynamics.h>

geometry_msgs::Pose
rovi_utils::make_pose(const std::array<double, 3>& pos, const Eigen::Quaternion<double>& ori)
{
	geometry_msgs::Pose pose;

	pose.position.x = pos[0];
	pose.position.y = pos[1];
	pose.position.z = pos[2];

	pose.orientation.w = ori.w();
	pose.orientation.x = ori.x();
	pose.orientation.y = ori.y();
	pose.orientation.z = ori.z();

	return pose;
}

geometry_msgs::Pose
rovi_utils::make_pose(const std::array<double, 6>& pose)
{
	return rovi_utils::make_pose
	(
		{ pose[0], pose[1], pose[2] },
		{ pose[3], pose[4], pose[5] }
	);
}

geometry_msgs::Pose
rovi_utils::make_pose(const std::array<double, 3>& pos, const std::array<double, 3>& rpy)
{
	geometry_msgs::Pose pose;

	pose.position.x = pos[0];
	pose.position.y = pos[1];
	pose.position.z = pos[2];

	Eigen::Quaterniond quat;

	quat = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
		   Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
		   Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) ;

	pose.orientation.w = quat.w();
	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();

	return pose;
}

moveit_msgs::CollisionObject
rovi_utils::make_mesh_cobj(const std::string& name, const std::string& planning_frame, const std::array<double, 3>& pos, const std::array<double, 4>& ori)
{
	// 3D model of <name> object is expected to be located at
	// package://rovi_gazebo/models/name/name.dae
	static const std::string PATH_PACKAGE = "package://rovi_gazebo/models";
	ROS_WARN_STREAM_ONCE("make_mesh_cobj() only searches '" << PATH_PACKAGE << "' path for .dae models.");

	// create mesh
	// https://answers.ros.org/question/246467/moveit-attach-object-error/
	
	auto model_name = name;
	model_name.erase(std::remove_if(model_name.begin(), model_name.end(), &isdigit), model_name.end());

	shape_msgs::Mesh mesh;
	shapes::Mesh* mesh_ptr;
	shapes::ShapeMsg shape_msg;

	mesh_ptr = shapes::createMeshFromResource(PATH_PACKAGE+ "/" + model_name + "/" + model_name + ".dae");
	shapes::constructMsgFromShape(mesh_ptr, shape_msg);
	mesh = boost::get<shape_msgs::Mesh>(shape_msg);

	// construct collision object

	moveit_msgs::CollisionObject co;

	co.id = name;
	co.meshes.resize(1);
	co.mesh_poses.resize(1);

	co.meshes[0] = mesh;
	co.header.frame_id = planning_frame;

	co.mesh_poses[0].position.x = pos[0];
	co.mesh_poses[0].position.y = pos[1];
	co.mesh_poses[0].position.z = pos[2];

	co.mesh_poses[0].orientation.w = ori[0];
	co.mesh_poses[0].orientation.x = ori[1];
	co.mesh_poses[0].orientation.y = ori[2];
	co.mesh_poses[0].orientation.z = ori[3];
	
	// co.pose.position.x = pos[0];
	// co.pose.position.y = pos[1];
	// co.pose.position.z = pos[2];

	// co.pose.orientation.w = ori[0];
	// co.pose.orientation.x = ori[1];
	// co.pose.orientation.y = ori[2];
	// co.pose.orientation.z = ori[3];

	// co.meshes.push_back(mesh);
	// co.mesh_poses.push_back(co.mesh_poses[0]);
	co.operation = co.ADD;

	return co;
}

void
rovi_utils::move_base(moveit::core::RobotState& state, const std::array<double, 3>& offset, const std::string& virtual_joint_name)
{
	// RobotState has a floating virtual joint, which can be set.
	// http://docs.ros.org/en/melodic/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html#ad08c92a61d43013714ec3894cd67a297

	// ROS_ERROR_STREAM(state.getRobotModel()->getRootJointName());
	// ROS_ERROR_STREAM(state.getRobotModel()->getRootJoint()->getTypeName());

	// the position of the virtual joint is set by calling
	// setVariablePositions(const std::map<std::string, double>& variable_map)

	ROS_DEBUG_STREAM("Moving base (joint: " << virtual_joint_name <<") to: { " << offset[0] << ", " << offset[2] << ", " << offset[2] << "}");

	state.setVariablePositions({
		{ virtual_joint_name + "/trans_x", offset[0] }, // state.setVariablePosition(0, offset[0]);
		{ virtual_joint_name + "/trans_y", offset[1] }, // state.setVariablePosition(1, offset[1]);
		{ virtual_joint_name + "/trans_z", offset[2] } // state.setVariablePosition(2, offset[2]);
	});
}

void
rovi_utils::move_base(moveit::core::RobotState& state, const geometry_msgs::Pose& offset, const std::string& virtual_joint_name)
{
	move_base(state, { offset.position.x, offset.position.y, offset.position.z }, virtual_joint_name);
}

template<typename T>
void
rovi_utils::export_traj(T& traj, const std::string&& filename, const double resolution)
{

	// MATLAB code:
	// data = readmatrix("traj_lin.csv");
	// plot3(data(:, 4), data(:, 8), data(:, 12))

	// get filename stem and extension
	const auto extension = boost::filesystem::path(filename).extension().string();
	const auto stem      = boost::filesystem::path(filename).stem().string();

	// KDL::Trajectory_Composite (Cartesian space)
	if constexpr (std::is_same<T, KDL::Trajectory_Composite>::value)
	{

		if (resolution <= 0)
			throw std::runtime_error("Resolution must be positive.");

		std::ofstream fs(filename, std::ofstream::out);

		for (double t = 0.0; t < traj.Duration(); t += resolution)
		{
			// KDL Frame
			const auto& frame = traj.Pos(t);
			const auto& twist = traj.Vel(t);

			// convert KDL Frame to Eigen
			static auto mat = Eigen::Affine3d();
			tf::transformKDLToEigen(frame, mat);

			// create flattened vector from Eigen (row major)
			auto T_ = mat.matrix();
			T_.transposeInPlace();
			Eigen::VectorXd v(Eigen::Map<Eigen::VectorXd>(T_.data(), T_.size()));

			for (size_t i = 0; i < v.size(); ++i)
				fs << v(i) << ((i != v.size() - 1) ? ", " : "");

			fs << "\n";
		}

		fs.close();
	}

	// robot_trajectory::RobotTrajectory (Cartesian space, waypoints)
	if constexpr (std::is_same<T, robot_trajectory::RobotTrajectory>::value)
	{
		auto robot_state = std::make_shared<moveit::core::RobotState>(*traj.getFirstWayPointPtr());
		ROS_WARN_STREAM("Exporting trajectory using waypoints; resolution (timestep) is ignored.");

		std::ofstream fs(filename, std::ofstream::out);

		for (size_t i = 0; i < traj.getWayPointCount(); ++i)
		{
			// auto robot_state = traj.getWayPoint(i);
			auto mat = traj.getWayPoint(i).getFrameTransform("ee_tcp");

			// create flattened vector from Eigen (row major)
			auto T_ = mat.matrix();
			T_.transposeInPlace();
			Eigen::VectorXd v(Eigen::Map<Eigen::VectorXd>(T_.data(), T_.size()));

			for (size_t i = 0; i < v.size(); ++i)
				fs << v(i) << ((i != v.size() - 1) ? ", " : "");

			fs << "\n";
		}

		fs.close();
	}

	// std::array<KDL::Trajectory_Composite*, 6> (Joint space)
	if constexpr (std::is_same<T, std::array<KDL::Trajectory_Composite*, 6>>::value)
	{
		if (resolution <= 0)
			throw std::runtime_error("Resolution must be positive.");

		std::ofstream fs_q(stem + "_q" + extension, std::ofstream::out);
		std::ofstream fs_qdot(stem + "_qdot" + extension, std::ofstream::out);

		const auto num_joints = traj.size();
		const auto max_dur = (*std::max_element(traj.begin(), traj.end(), [&](auto& a, auto& b) {
			return a->Duration() < b->Duration();
		}))->Duration();

		for (double t = 0.0; t < max_dur; t += resolution)
		{
			std::string str_q    = "";
			std::string str_qdot = "";

			for (size_t i = 0; i < num_joints; ++i)
			{
				double q = traj[i]->Pos(t).p.data[0];
				double qdot = traj[i]->Vel(t).vel.data[0];

				str_q    +=    (str_q.empty() ? "" : ", ") + std::to_string(q);
				str_qdot += (str_qdot.empty() ? "" : ", ") + std::to_string(qdot);
			}

			fs_q    << str_q    << "\n";
			fs_qdot << str_qdot << "\n";
		}

		fs_q.close();
		fs_qdot.close();
	}
}

std::vector<geometry_msgs::Pose>
rovi_utils::waypoints_from_traj(const robot_trajectory::RobotTrajectory& traj)
{
	if (not traj.getWayPointCount())
		throw std::runtime_error("Trajectory is empty.");

	std::vector<geometry_msgs::Pose> waypoints;

	for (size_t i = 0; i < traj.getWayPointCount(); ++i)
	{
		// const auto mat = traj.getWayPoint(i).getFrameTransform("ur5_link6");
		const auto mat = traj.getWayPoint(i).getFrameTransform("ee_tcp");
		const auto t   = mat.translation() - traj.getWayPoint(i).getFrameTransform("ur5_link0").translation();
		const auto r   = mat.rotation();

		// create pose from translation and rotation; add to vector
		waypoints.emplace_back(make_pose( { t[0], t[1], t[2] }, Eigen::Quaternion<double>(r)));
	}

	return waypoints;
}

std::vector<sensor_msgs::JointState>
rovi_utils::joint_states_from_traj(const robot_trajectory::RobotTrajectory& traj)
{
	if (not traj.getWayPointCount())
		throw std::runtime_error("Trajectory is empty.");

	std::vector<sensor_msgs::JointState> joint_states;

	static std::ofstream fs("write_to_file.csv", std::ofstream::out);

	for (size_t i = 0; i < traj.getWayPointCount(); ++i)
	{
		std::vector<double> q, qdot, qddot;
		sensor_msgs::JointState joint_state;

		// get joint values, define joint statee and add to vector of states
		traj.getWayPoint(i).copyJointGroupPositions("ur5_arm", q);
		joint_state.position = q;

		traj.getWayPoint(i).copyJointGroupVelocities("ur5_arm", qdot);
		joint_state.velocity = qdot;

		traj.getWayPoint(i).copyJointGroupVelocities("ur5_arm", qddot);
		joint_state.effort = qddot;
			
		joint_states.push_back(joint_state);

		fs << joint_state.position[0] << ", " << joint_state.position[1] << ", " <<  joint_state.position[2] << ", " << joint_state.position[3] << ", " << joint_state.position[4] << ", " << joint_state.position[5] << ", " << std::endl;
	}

	fs.close();

	return joint_states;
}

// rovi_utils::export_traj(const T& traj, const std::string&& filename, const double resolution)
template void rovi_utils::export_traj<KDL::Trajectory_Composite>(KDL::Trajectory_Composite& traj, const std::string&& filename, const double resolution);
template void rovi_utils::export_traj<robot_trajectory::RobotTrajectory>(robot_trajectory::RobotTrajectory& traj, const std::string&& filename, const double resolution);
template void rovi_utils::export_traj<std::array<KDL::Trajectory_Composite*, 6>>(std::array<KDL::Trajectory_Composite*, 6>& traj, const std::string&& filename, const double resolution);
