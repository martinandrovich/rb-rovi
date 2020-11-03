#include<ur5_controllers/wsg_hybrid_controller.h>

namespace ur5_controllers
{
bool
WSGHybridController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
{
	ROS_ERROR("Init is inited");

    // get list of joints (from parameter server)
	if (!nh.getParam("joint_names", vec_joint_names))
	{
		ROS_ERROR_NAMED(CONTROLLER_NAME, "No joints were specifed.");
		return false;
	}

	// get number of joints; exit if zero
	if (num_joints = vec_joint_names.size(); num_joints == 0)
	{
		ROS_ERROR_NAMED(CONTROLLER_NAME, "Vector of joint names is empty.");
		return false;
	}

	// fill vector of joint handles
	for (const auto& joint_name : vec_joint_names)
	{
		try
		{
			vec_joints.push_back(hw->getHandle(joint_name));
		}
		catch (const hardware_interface::HardwareInterfaceException& e)
		{
			ROS_ERROR_NAMED(CONTROLLER_NAME, "Error getting joint handles: %s", e.what());
			return false;
		}
	}

	// initialize command buffer and q_d vector
	commands_buffer.writeFromNonRT(0);
	q_d = Eigen::Vector2d::Zero();

	// subscribe to joint position command
	sub_command = nh.subscribe<std_msgs::Float64>("command", 1, &WSGHybridController::callback_command, this);

	// init complete
	ROS_INFO_STREAM_NAMED(CONTROLLER_NAME, "Loaded " << CONTROLLER_NAME << " with kp = " << kp << ", kd = " << kd);
	return true;
}

void
WSGHybridController::starting(const ros::Time& time)
{
    // Implement PID loop
    for(int i = 0; i < 2; i++)
    {
        vec_joints[i].setCommand(0.0);
    }
	ROS_INFO_ONCE("Updating WSGHybridController Loop");
}

void
WSGHybridController::update(const ros::Time& time, const ros::Duration& dur)
{
    // Implement PID loop
	//

	/*
		- Pseudo Code

		if state = no torque, then make slowly go the opposite way
		with some decremental torque.

		if  state = torque, then activate position control until 
			there is no movement, this is supposed to be a very slow integral torque force controller, that is,
			make sure the torque reaches some specified value, remember to use the force-torque sensor to read this..

	*/

    for(int i = 0; i < 2; i++)
    {
        vec_joints[i].setCommand(100);
    }
	ROS_INFO_ONCE("Updating WSGHybridController Loop");
}

Eigen::Vector2d 
WSGHybridController::saturate_rotatum(const Eigen::Vector2d& tau_des, const double period /* [s] */)
{
	// make sure this saturates how fast the torque changes
}

void
WSGHybridController::callback_command(const std_msgs::Float64ConstPtr& msg)
{
	// set some specified torque.
}

}

// export controller
PLUGINLIB_EXPORT_CLASS(ur5_controllers::WSGHybridController, controller_interface::ControllerBase)