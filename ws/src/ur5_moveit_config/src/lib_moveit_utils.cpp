#include "moveit_utils/moveit_utils.h"

#include <ros/ros.h>

moveit_msgs::CollisionObject
moveit::make_mesh_cobj(const std::string& name, const std::array<double, 3>& pos, const std::array<double, 4>& ori)
{
	// 3D model of <name> object is expected to be located at
	// package://rovi_gazebo/models/name/name.dae
	static const std::string PATH_PACKAGE = "package://rovi_gazebo/models";
	ROS_WARN_STREAM_ONCE("moveit::make_mesh_cobj() only searches the" << PATH_PACKAGE << " path for .dae models.");

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
	co.header.frame_id = "base_link";

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