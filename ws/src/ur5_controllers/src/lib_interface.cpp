#include "ur5_controllers/interface.h"

void
ur5_controllers::ur5::init()
{
	ROS_INFO_STREAM("Creating private ~wsg_cmd_publisher node handle (once)...");
	
	nh = new ros::NodeHandle("~wsg_cmd_publisher");
	pub_cmd = nh->advertise<std_msgs::Float64>(COMMAND_TOPIC, 1);
}

void
ur5_controllers::ur5::execute_traj(const std::vector<sensor_msgs::JointState>& traj, const double freq)
{

	if (not nh)
		init();

	ROS_INFO_STREAM("Commanding UR5 joint rajectory (position controller) at " << freq << " [Hz]...");

	ros::Rate lp(freq); // Hz
	for (const auto& msg : traj)
	{
		pub_cmd.publish(msg);
		lp.sleep();
	}
}

void
ur5_controllers::wsg::init()
{
	ROS_INFO_STREAM("Creating private ~wsg_cmd_publisher node handle (once)...");
	
	nh = new ros::NodeHandle("~wsg_cmd_publisher");
	pub_cmd = nh->advertise<std_msgs::Float64>(COMMAND_TOPIC, 1);

	thread_pub = new std::thread([&]()
	{	
		ros::Rate lp(PUB_FREQ);
		std_msgs::Float64 wsg_msg;
		while (ros::ok())
		{
			wsg_msg.data = tau_des;
			pub_cmd.publish(wsg_msg);
			lp.sleep();
		}
	});
}

void
ur5_controllers::wsg::grasp()
{
	if (not thread_pub)
		init();
	
	else
		tau_des = EFFORT_GRASP;
}

void
ur5_controllers::wsg::release()
{
	if (not thread_pub)
		init();
	
	else
		tau_des = EFFORT_RELEASE;
}