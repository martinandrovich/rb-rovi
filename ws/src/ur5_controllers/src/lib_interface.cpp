#include "ur5_controllers/interface.h"

#include <kdl_conversions/kdl_msg.h>
#include <ur5_controllers/PoseTwist.h>

void
ur5_controllers::ur5::check_nh_init()
{
	if (not nh)
	{
		nh = new ros::NodeHandle("~ur5_cmd_publisher");
		ROS_WARN_STREAM("Creating private ~ur5_jnt_pos_cmd_publisher node handle (once)...");
	}
}

void
ur5_controllers::ur5::execute_traj(const std::vector<sensor_msgs::JointState>& traj, const double freq)
{
	ur5_controllers::ur5::check_nh_init();
	if (not pub_cmd_jnt_pos)
		pub_cmd_jnt_pos = std::make_shared<ros::Publisher>(nh->advertise<sensor_msgs::JointState>(COMMAND_JNT_POS_TOPIC, 1));

	ROS_INFO_STREAM("Commanding UR5 joint trajectory at " << freq << " [Hz]...");

	ros::Rate lp(freq); // Hz
	for (const auto& msg : traj)
	{
		pub_cmd_jnt_pos->publish(msg);
		lp.sleep();
	}
}

void
ur5_controllers::ur5::execute_traj(const KDL::Trajectory_Composite* traj, const double freq)
{
	ur5_controllers::ur5::check_nh_init();
	if (not pub_cmd_cart)
		pub_cmd_cart = std::make_shared<ros::Publisher>(nh->advertise<ur5_controllers::PoseTwist>(COMMAND_CART_TOPIC, 1));

	ROS_INFO_STREAM("Commanding UR5 Cartesian trajectory at " << freq << " [Hz]...");

	ros::Rate lr(freq); // [Hz]
	ur5_controllers::PoseTwist msg;
	for (double t = 0.0; t < traj->Duration() and ros::ok(); t += 1/freq)
	{
		// KDL Frame
		const auto& frame = traj->Pos(t);
		const auto& twist = traj->Vel(t);

		// Convert from KDL to Eigen
		tf::poseKDLToMsg(frame, msg.pose);
		tf::twistKDLToMsg(twist, msg.twist);

		pub_cmd_cart->publish(msg);
		lr.sleep();
	}
}

void
ur5_controllers::wsg::init()
{
	ROS_WARN_STREAM("Creating private ~wsg_cmd_publisher node handle (once)...");

	nh = new ros::NodeHandle("~wsg_cmd_publisher");
	pub_cmd = nh->advertise<std_msgs::Float64>(COMMAND_TOPIC, 1);

	thread_pub = new std::thread([&]()
	{
		ros::Rate lr(PUB_FREQ);
		std_msgs::Float64 wsg_msg;
		while (ros::ok())
		{
			wsg_msg.data = tau_des;
			pub_cmd.publish(wsg_msg);
			lr.sleep();
		}
	});
	
	thread_pub->detach();
}

void
ur5_controllers::wsg::grasp(bool wait)
{
	if (not thread_pub)
		wsg::init();

	tau_des = EFFORT_GRASP;
}

void
ur5_controllers::wsg::release(bool wait)
{
	if (not thread_pub)
		wsg::init();

	tau_des = EFFORT_RELEASE;
}