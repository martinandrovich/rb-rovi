#include <iostream>
#include <vector>
#include <tuple>

#include <ros/ros.h>
#include <rovi_planner/rovi_planner.h>
#include <kdl_conversions/kdl_msg.h>

#include <ur5_controllers/PoseTwist.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>

#include <rovi_utils/rovi_utils.h>

int
main(int argc, char** argv)
{
	// check input arguments
	if (argc < 5)
	{
		std::cerr << "Usage: goto_pose mode dx dy dz\n";
		std::cout << "\n"
		             "The pose is given in the world frame. Mode can be set to 'rel' or 'abs'.\n\n"
		             "Example: goto_pose rel 0.1 -0.2 0\n";
		exit(-1);
	}

	// init node
	ros::init(argc, argv, "goto_pose");
	ros::NodeHandle nh;

	// define changes in x, y, z from arguments as doubles
	const auto mode = std::string(argv[1]);
	const auto [dx, dy, dz] = std::tuple{ std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]) };

	if (mode != "abs" and mode != "rel")
	{
		std::cerr << "mode of type " << mode << " is not recognized; please use abs or rel\n";
		exit(-1);
	}

	// get link states from gazebo
	ROS_INFO_STREAM("Getting link_states from Gazebo...");
	// const auto& link_states = ros::topic::waitForMessage<gazebo_msgs::LinkStates>("/gazebo/link_states");
	
	gazebo_msgs::LinkStates* link_states = nullptr;
	const auto sub_link_states = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, [&](const auto& msg) {
		ROS_WARN("Got LinkStates in temporary ROS callback.");
		link_states = new gazebo_msgs::LinkStates(*msg);
	});

	while (not link_states)
		ros::spinOnce();

	// get current pose of base and link6 in world frame
	const auto& link_names = link_states->name;
	size_t idx_base  = std::distance(link_names.begin(), std::find(link_names.begin(), link_names.end(), "ur5::ur5_link0"));
	size_t idx_link6 = std::distance(link_names.begin(), std::find(link_names.begin(), link_names.end(), "ur5::ur5_link6"));
	
	const auto offset = link_states->pose[idx_base];
	auto pose_start   = link_states->pose[idx_link6];

	// controller uses pose given in robot base frame; offset from world
	pose_start.position.x -= offset.position.x;
	pose_start.position.y -= offset.position.y;
	pose_start.position.z -= offset.position.z;

	ROS_INFO_STREAM("Start pose:\n\n" << pose_start << "\n");

	// set desired pose
	double pos_x, pos_y, pos_z;
	if (mode == "rel")
	{
		pos_x = pose_start.position.x + dx;
		pos_y = pose_start.position.y + dy;
		pos_z = pose_start.position.z + dz;
	}
	else
	if (mode == "abs")
	{
		pos_x = dx;
		pos_y = dy;
		pos_z = dz;
	}
	
	const auto pose_end = rovi_utils::make_pose({ pos_x, pos_y, pos_z }, { pose_start.orientation.w, pose_start.orientation.x, pose_start.orientation.y, pose_start.orientation.z } );
	ROS_INFO_STREAM("End pose:\n\n" << pose_end << "\n");

	// define waypoints
	std::vector<geometry_msgs::Pose> waypoints = 
	{
		pose_start,
		pose_end
	};

	// generate trajectory
	auto traj = rovi_planner::traj_parabolic(waypoints, 0.1, 0.1, 0.01, 0.01);
	ROS_INFO_STREAM("Generated trajectory with duration: " << traj.Duration() << " sec");

	// publish
	const auto pub_cmd = nh.advertise<ur5_controllers::PoseTwist>("/ur5_cartesian_pose_controller/command", 1);

	constexpr auto LOOP_FREQ = 100.; // Hz
	ros::Rate loop_rate(LOOP_FREQ);
	ur5_controllers::PoseTwist msg;
	for (double t = 0.0; t < traj.Duration(); t += 1/LOOP_FREQ)
	{
		const auto& frame = traj.Pos(t);
		const auto& twist = traj.Vel(t);

		tf::poseKDLToMsg(frame, msg.pose);
		tf::twistKDLToMsg(twist, msg.twist);

		pub_cmd.publish(msg);
		loop_rate.sleep();
	}

	return 0;
}