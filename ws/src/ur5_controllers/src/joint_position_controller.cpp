#include <pluginlib/class_list_macros.hpp>
#include <ur5_controllers/joint_position_controller.h>

namespace ur5_controllers
{

	bool
	JointPositionController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
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

		// subscribe to joint position command
		sub_command = nh.subscribe<sensor_msgs::JointState>("command", 1, &JointPositionController::callback_command, this);

		// init complete
		ROS_INFO_STREAM_NAMED(CONTROLLER_NAME, "Loaded " << CONTROLLER_NAME << " with kp = " << kp << ", kd = " << kd);
		return true;
	}

	void
	JointPositionController::starting(const ros::Time& time)
	{
		sensor_msgs::JointState q_state;

		// initial desired position
		for (size_t i = 0; i < Q_D_INIT.size(); i++)
		{
			q_d(i) = Q_D_INIT[i];
			q_dot_d(i) = 0.0f;
			//q_ddot_d(i) = 0.0f;
		}

		q_state.position = std::vector<double>(q_d.data(), q_d.data() + q_d.size());
		q_state.velocity = std::vector<double>(q_dot_d.data(), q_dot_d.data() + q_dot_d.size());
		//q_state.effort = std::vector<double>(q_ddot_d.data(), q_ddot_d.data() + q_ddot_d.size());

		commands_buffer.writeFromNonRT(q_state);
	}

	void
	JointPositionController::update(const ros::Time& /*time*/, const ros::Duration& period)
	{
		// elapsed time
		static ros::Duration elapsed_time = ros::Duration(0.);
		elapsed_time += period;

		// get desired joint efforts
		const auto & command = *commands_buffer.readFromRT();

		// write the commanded velocity in joint space
		q_d 	= Eigen::Vector6d(command.position.data());
		q_dot_d = Eigen::Vector6d(command.velocity.data());
		//q_ddot_d = Eigen::Vector6d(command.effort.data());


		// read joint states
		const auto q = get_position();
		const auto q_dot = get_velocity();

		// compute dynamics (via KDL)
		const auto g = ur5_dynamics::gravity(q);
		const auto m = ur5_dynamics::mass(q);
		const auto c = ur5_dynamics::coriolis(q, q_dot);

		Eigen::Vector6d tau_des;
		{
			Eigen::DiagonalMatrix<double, 6, 6> kp_m;
			kp_m.diagonal() << kp, kp, kp, kp, kp, kp;

			Eigen::DiagonalMatrix<double, 6, 6> kd_m;
			kd_m.diagonal() << kd, kd, kd, kd, kd, kd;

			const auto y = m * ( kp_m * ( q_d - q ) + kd_m * ( q_dot_d - q_dot ) );

			tau_des = y + g + c;
		}

		// saturate rate-of-effort (rotatum) this works life a real-life factor xD
		// if (SATURATE_ROTATUM)
		//     tau_des = saturate_rotatum(tau_des, period.toSec());
		

		// set desired command on joint handles
		for (size_t i = 0; i < num_joints; ++i)
			vec_joints[i].setCommand(tau_des[i]);
	}

	Eigen::Vector6d
	JointPositionController::get_position()
	{
		static Eigen::Vector6d q;

		for (size_t i = 0; i < vec_joints.size(); ++i)
			q[i] = vec_joints[i].getPosition();

		return q;
	}

	Eigen::Vector6d
	JointPositionController::get_velocity()
	{
		static Eigen::Vector6d qdot;

		for (size_t i = 0; i < vec_joints.size(); ++i)
			qdot[i] = vec_joints[i].getVelocity();

		return qdot;
	}

	Eigen::Vector6d
	JointPositionController::saturate_rotatum(const Eigen::Vector6d& tau_des, const double period)
	{
		// previous desired torque and saturated torque
		static Eigen::Vector6d tau_des_prev = Eigen::Vector6d::Zero();
		static Eigen::Vector6d tau_des_sat = Eigen::Vector6d::Zero();

		// compute saturated torque
		for (size_t i = 0; i < tau_des_sat.size(); ++i)
		{
			const double tau_dot = (tau_des[i] - tau_des_prev[i]) / period;
			tau_des_sat[i] = tau_des_prev[i] + std::max(std::min(tau_dot, TAU_DOT_MAX * period), -(TAU_DOT_MAX * period));
		}

		// save for next iteration and return
		tau_des_prev = tau_des_sat;
		return tau_des_sat;
	}

	void
	JointPositionController::callback_command(const sensor_msgs::JointStateConstPtr& msg)
	{
		// write commands to command buffer
		commands_buffer.writeFromNonRT(*msg);
	}

}

// export controller
PLUGINLIB_EXPORT_CLASS(ur5_controllers::JointPositionController, controller_interface::ControllerBase)