#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>

#include <ros/ros.h>
#include <kdl/trajectory_composite.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

namespace ur5_controllers
{

	class ur5
	{

	public:

		ur5() = delete;

		static void
		execute_traj(const std::vector<sensor_msgs::JointState>& traj, const double freq = EXEC_FREQ);
		
		static void
		execute_traj(const KDL::Trajectory_Composite* traj, const double freq = EXEC_FREQ);

		inline static const std::string COMMAND_JNT_POS_TOPIC = "/ur5_joint_position_controller/command";
		inline static const std::string COMMAND_CART_TOPIC    = "/ur5_cartesian_pose_controller/command";
		inline static constexpr auto    EXEC_FREQ             = 1000.0; // [Hz]


	private:

		static void
		check_nh_init();

		static inline ros::NodeHandle* nh;
		static inline std::shared_ptr<ros::Publisher> pub_cmd_jnt_pos;
		static inline std::shared_ptr<ros::Publisher> pub_cmd_cart;

	};

	class wsg
	{

	public:

		wsg() = delete;

		static void
		grasp(bool wait = false);

		static void
		release(bool wait = false);

		static inline const std::string	COMMAND_TOPIC  = "/wsg_hybrid_controller/command";
		static inline const double      PUB_FREQ       = 100.0; // [Hz]
		static inline const double      EFFORT_GRASP   = -50.0; // [Nm]
		static inline const double      EFFORT_RELEASE =  30.0; // [Nm]

	private:

		static void
		init();

		static inline ros::NodeHandle* nh;
		static inline ros::Publisher pub_cmd;
		static inline std::thread* thread_pub = nullptr;
		// static inline std::mutex mutex_cmd;
		static inline std::atomic<double> tau_des;

	};
}