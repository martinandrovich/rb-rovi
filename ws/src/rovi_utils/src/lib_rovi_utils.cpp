#include "rovi_utils/rovi_utils.h"

#include <thread>
#include <pthread.h>

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometric_shapes/shape_operations.h>
#include <tf/transform_broadcaster.h>

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

// geometry_msgs::Pose
// rovi_utils::make_pose(const std::array<double, 3>& pos, const std::array<double, 4>& ori)
// {
// 	geometry_msgs::Pose pose;

// 	pose.position.x = pos[0];
// 	pose.position.y = pos[1];
// 	pose.position.z = pos[2];

// 	pose.orientation.w = ori[0];
// 	pose.orientation.x = ori[1];
// 	pose.orientation.y = ori[2];
// 	pose.orientation.z = ori[3];

// 	return pose;
// }

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
rovi_utils::make_mesh_cobj(const std::string& name, const std::string& frame, const std::array<double, 3>& pos, const std::array<double, 4>& ori)
{
	// 3D model of <name> object is expected to be located at
	// package://rovi_gazebo/models/name/name.dae
	static const std::string PATH_PACKAGE = "package://rovi_gazebo/models";
	ROS_WARN_STREAM_ONCE("make_mesh_cobj() only searches '" << PATH_PACKAGE << "' path for .dae models.");

	// create mesh
	// https://answers.ros.org/question/246467/moveit-attach-object-error/

	shape_msgs::Mesh mesh;
	shapes::Mesh* mesh_ptr;
	shapes::ShapeMsg shape_msg;

	mesh_ptr = shapes::createMeshFromResource(PATH_PACKAGE+ "/" + name + "/" + name + ".dae");
	shapes::constructMsgFromShape(mesh_ptr, shape_msg);
	mesh = boost::get<shape_msgs::Mesh>(shape_msg);

	// construct collision object

	moveit_msgs::CollisionObject co;

	co.id = name;
	co.meshes.resize(1);
	co.mesh_poses.resize(1);

	co.meshes[0] = mesh;
	co.header.frame_id = frame;

	co.mesh_poses[0].position.x = pos[0];
	co.mesh_poses[0].position.y = pos[1];
	co.mesh_poses[0].position.z = pos[2];

	co.mesh_poses[0].orientation.w = ori[0];
	co.mesh_poses[0].orientation.x = ori[1];
	co.mesh_poses[0].orientation.y = ori[2];
	co.mesh_poses[0].orientation.z = ori[3];

	co.meshes.push_back(mesh);
	co.mesh_poses.push_back(co.mesh_poses[0]);
	co.operation = co.ADD;

	return co;
}

std::vector<moveit_msgs::CollisionObject>
rovi_utils::get_gazebo_obj(const std::string& frame, const std::vector<std::string>& excludes)
{
	// get vector of all objects in gazebo as ModelState msgs
	ROS_INFO_STREAM("Waiting for gazebo_msgs::ModelStates...");
	const auto& model_states = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
	const auto& vec_poses = model_states->pose;
	const auto& vec_obj = model_states->name;

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

			// construct and add colision object
			ROS_INFO_STREAM("Adding object: " << obj << " at " << "[" << pos.x << ", " << pos.y << ", " << pos.z << "]");
			collision_objects.emplace_back(make_mesh_cobj(obj, frame, { pos.x, pos.y, pos.z }));
		}
		else
			ROS_WARN_STREAM("Excluding object: " << obj);
	}

	return collision_objects;
}

void
rovi_utils::move_base(const std::string& frame_id, const std::string& child, const std::array<double, 3>& pos)
{	

	static std::thread * thread = nullptr;

	if (thread != nullptr)
	{
		auto handle = thread->native_handle();
		pthread_cancel(handle);
	}

	thread = new std::thread([=]()
	{
		constexpr auto FREQ = 100.; // Hz
		tf::TransformBroadcaster broadcaster;
		ros::Rate lp(FREQ); // Hz
		
		while (ros::ok())
		{
			tf::StampedTransform transform = tf::StampedTransform
			(
				tf::Transform( tf::Quaternion(0., 0., 0., 1.),
				tf::Vector3(pos[0], pos[1], pos[2])),
				ros::Time().now() + ros::Duration(1./FREQ),
				frame_id,
				child
			);

			broadcaster.sendTransform(transform);
			lp.sleep();
		}
	});
}