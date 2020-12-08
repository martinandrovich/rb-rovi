#include <ur5_controllers/wsg_hybrid_controller.h>

namespace ur5_controllers
{
	bool
	WSGHybridController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
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
		{
			std_msgs::Float64 msg; msg.data = 0;
			commands_buffer.writeFromNonRT(msg);
		}

		// init ori_ee_buffer
		{
			geometry_msgs::Pose msg;

			geometry_msgs::Point pos; 
			pos.x = 0.f; 
			pos.y = 0.f; 
			pos.z = 0.f;

			geometry_msgs::Quaternion ori; 
			ori.x = 0.f; 
			ori.y = 0.f; 
			ori.z = 0.f; 
			ori.w = 1.f;

			msg.position = pos; msg.orientation = ori;
			ori_ee_buffer.writeFromNonRT(msg);
		}

		// subscribe to joint position command
		sub_command = nh.subscribe<std_msgs::Float64>("command", 1, &WSGHybridController::callback_command, this);

		// subscribe to pose you cannot have two subscribers on the same...
		ori_ee = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, &WSGHybridController::callback_ori, this);

		// init complete
		ROS_INFO_STREAM_NAMED(CONTROLLER_NAME, "Loaded " << CONTROLLER_NAME);
		return true;
	}

	void
	WSGHybridController::starting(const ros::Time& time)
	{
		// Implement PID loop
		for (size_t i = 0; i < 2; i++)
		{
			vec_joints[i].setCommand(0);
		}

		ROS_INFO_ONCE("Start WSG controller");
	}

	void
	WSGHybridController::update(const ros::Time& time, const ros::Duration& dur)
	{
		// get desired joint efforts
		const auto& command = *commands_buffer.readFromRT();

		// Eigen::Vector2d tau_des(command.data, command.data);

		// Eigen::Vector2d q = get_position();

		// const auto& ori = ori_ee_buffer.readFromNonRT()->orientation;
		// Eigen::Quaterniond quat{ori.w, ori.x, ori.y, ori.z};
		// Eigen::Matrix3d R = quat.toRotationMatrix();
		// Eigen::Vector3d g = R * Eigen::Vector3d(0, 0, 1.5*9.80665);

		// for (size_t i = 0; i < 2; i++)
		// 	tau_des(i) += (i == 0) ? g(0) : -g(0);
		
		// saturate rotatum
		// tau_des = saturate_rotatum(tau_des);

		for (size_t i = 0; i < 2; i++)
			vec_joints[i].setCommand(command.data);
			// vec_joints[i].setCommand(tau_des(i));
	}

	Eigen::Vector2d
	WSGHybridController::saturate_rotatum(const Eigen::Vector2d& tau_des, const double period)
	{
		// previous desired torque and saturated torque
		static Eigen::Vector2d tau_des_prev = Eigen::Vector2d::Zero();
		static Eigen::Vector2d tau_des_sat = Eigen::Vector2d::Zero();

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
	WSGHybridController::callback_ori(const gazebo_msgs::LinkStatesConstPtr& msg)
	{
		// callback orientation
		geometry_msgs::Pose pose(msg->pose[6]);
		ori_ee_buffer.writeFromNonRT(pose);
	}

	void
	WSGHybridController::callback_command(const std_msgs::Float64ConstPtr& msg)
	{
		// set some specified torque.
		commands_buffer.writeFromNonRT(*msg);
	}

	Eigen::Vector2d
	WSGHybridController::get_position()
	{
		static Eigen::Vector2d q;

		for (size_t i = 0; i < vec_joints.size(); ++i)
			q[i] = vec_joints[i].getPosition();

		return q;
	}

}

// export controller
PLUGINLIB_EXPORT_CLASS(ur5_controllers::WSGHybridController, controller_interface::ControllerBase)