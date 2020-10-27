#include <ur5_controllers/joint_position_pd_gravity_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace ur5_controllers
{

bool
JointPositionPDGravityController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
{
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
	commands_buffer.writeFromNonRT(std::vector<double>(num_joints, 0.0));
	q_d = Eigen::Vector6d::Zero();

	// subscribe to joint position command
	sub_command = nh.subscribe<std_msgs::Float64MultiArray>("command", 1, &JointPositionPDGravityController::callback_command, this);

	// init complete
	ROS_INFO_STREAM_NAMED(CONTROLLER_NAME, "Loaded " << CONTROLLER_NAME << " with kp = " << kp << ", kd = " << kd);
	return true;
}

void
JointPositionPDGravityController::starting(const ros::Time& time)
{
	// initial desired position
	// commands_buffer.readFromRT()->assign(num_joints, 0.);
	commands_buffer.writeFromNonRT(Q_D_INIT);
}

void 
JointPositionPDGravityController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
	// elapsed time
	static ros::Duration elapsed_time = ros::Duration(0.);
	elapsed_time += period;

	// get desired joint efforts
	const std::vector<double>& commands = *commands_buffer.readFromRT();
	for (size_t i = 0; i < commands.size(); ++i)
		q_d[i] = commands[i];

	// read joint states
	const auto q    = get_position();
	const auto qdot = get_velocity();

	// compute dynamics (via KDL)
	const auto g = ur5_dynamics::gravity(q);

	// compute controller effort
	Eigen::Vector6d tau_des = kp * (q_d - q) - kd * qdot + g;

	// saturate rate-of-effort (rotatum)
	if (SATURATE_ROTATUM)
		tau_des = saturate_rotatum(tau_des, period.toSec());

	// set desired command on joint handles
	for (size_t i = 0; i < num_joints; ++i)
		vec_joints[i].setCommand(tau_des[i]);
}

Eigen::Vector6d
JointPositionPDGravityController::get_position()
{
	static Eigen::Vector6d q;

	for (size_t i = 0; i < vec_joints.size(); ++i)
		q[i] = vec_joints[i].getPosition();

	return q;
}

Eigen::Vector6d
JointPositionPDGravityController::get_velocity()
{
	static Eigen::Vector6d qdot;

	for (size_t i = 0; i < vec_joints.size(); ++i)
		qdot[i] = vec_joints[i].getVelocity();

	return qdot;
}

Eigen::Vector6d
JointPositionPDGravityController::saturate_rotatum(const Eigen::Vector6d& tau_des, const double period)
{
	// previous desired torque and saturated torque
	static Eigen::Vector6d tau_des_prev = Eigen::Vector6d::Zero();
	static Eigen::Vector6d tau_des_sat  = Eigen::Vector6d::Zero();
	
	// compute saturated torque
	for (size_t i = 0; i < tau_des_sat.size(); ++i)
	{
		// const double diff = tau_des[i] - tau_des_prev[i];
		// tau_des_sat[i] = tau_des_prev[i] + std::max(std::min(diff, 1.0), -1.0);
		const double tau_dot = (tau_des[i] - tau_des_prev[i]) / period;
		tau_des_sat[i] = tau_des_prev[i] + std::max(std::min(tau_dot, TAU_DOT_MAX * period), -(TAU_DOT_MAX * period));
	}

	// save for next iteration and return
	tau_des_prev = tau_des_sat;
	return tau_des_sat;
}

void
JointPositionPDGravityController::callback_command(const std_msgs::Float64MultiArrayConstPtr& msg)
{
	// check size
	if (msg->data.size() != num_joints)
	{
		ROS_ERROR_NAMED(CONTROLLER_NAME, "Number of desired values in command (%lu) does not match number of joints (%lu); execution aborted.", msg->data.size(), num_joints);
		return;
	}

	// write commands to command buffer
	commands_buffer.writeFromNonRT(msg->data);
}

}

// export controller
PLUGINLIB_EXPORT_CLASS(ur5_controllers::JointPositionPDGravityController, controller_interface::ControllerBase)