#include "moveit_utils/moveit_utils.h"

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

moveit_msgs::CollisionObject
moveit::make_mesh_cobj(const std::string& name, const std::string& frame, const std::array<double, 3>& pos, const std::array<double, 4>& ori)
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
moveit::get_gazebo_obj(const std::string& frame, const std::vector<std::string>& excludes)
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
moveit::move_base(const std::string& frame_id, const std::string& child, const std::array<double, 3>& pos)
{	

	static std::thread * threadptr = nullptr;

	if (threadptr != nullptr)
	{
		auto handle = threadptr->native_handle();
		pthread_cancel(handle);
	}

	threadptr = new std::thread([=]()
	{
		const double time = 100;
		static tf::TransformBroadcaster broadcaster;
		ros::Rate lp(time);
		
		while (ros::ok())
		{
			tf::StampedTransform transform = tf::StampedTransform
			(
				tf::Transform( tf::Quaternion(0., 0., 0., 1.),
				tf::Vector3(pos[0], pos[1], pos[2])),
				ros::Time().now() + ros::Duration(1./time),
				frame_id,
				child
			);
			broadcaster.sendTransform(transform);
			lp.sleep();
		}
	});
}
