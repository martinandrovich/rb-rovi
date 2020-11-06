#pragma once

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <ur5_dynamics/ur5_dynamics.h>
#include <ur5_controllers/PoseTwist.h>
#include <urdf/model.h>
#include <wsg_dynamics/wsg_dynamics.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ur5_controllers
{

    class WSGHybridController final: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:

            static inline constexpr auto CONTROLLER_NAME = "WSGHybridController";
            static inline constexpr auto SATURATE_ROTATUM = true;
            static inline constexpr auto TAU_DOT_MAX = 100.;
            static inline const std::vector<double> Q_D_INIT = { 0.f, 0.f };

            std::vector<std::string> vec_joint_names;
            size_t num_joints;

            std::vector<hardware_interface::JointHandle> vec_joints;
            realtime_tools::RealtimeBuffer<std_msgs::Float64> commands_buffer;
            realtime_tools::RealtimeBuffer<geometry_msgs::Pose> ori_ee_buffer;

            //Default Constructor
            WSGHybridController() {}

            ~WSGHybridController() { sub_command.shutdown(); }

            bool
            init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;

            void
            starting(const ros::Time& time) override;

            void
            update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

        private:

            ros::Subscriber sub_command;
            ros::Subscriber ori_ee;

            Eigen::Vector2d 
            saturate_rotatum(const Eigen::Vector2d& tau_des, const double period = 0.001 /* [s] */);

            Eigen::Vector2d get_position();

            void
            callback_ori(const gazebo_msgs::LinkStatesConstPtr& msg);

            void
            callback_command(const std_msgs::Float64ConstPtr& msg);
    };

}
