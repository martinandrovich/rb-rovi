#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

namespace ur5_controllers
{
	class ur5
	{

	public:

		ur5() = delete;

		static void
		execute_traj(const std::vector<sensor_msgs::JointState>& traj, const double freq = 100.);

		inline static const std::string COMMAND_TOPIC = "/ur5_joint_position_controller/command";

	private:

		static void
		init();

		static inline ros::NodeHandle* nh;
		static inline ros::Publisher pub_cmd;

	};

	class wsg
	{

	public:

		wsg() = delete;

		static void
		grasp();

		static void
		release();

		static inline const std::string	COMMAND_TOPIC  = "/wsg_hybrid_controller/command";
		static inline const double      PUB_FREQ       = 100.; // Hz
		static inline const double      EFFORT_GRASP   = -30.; // Nm
		static inline const double      EFFORT_RELEASE =  30.; // Nm

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