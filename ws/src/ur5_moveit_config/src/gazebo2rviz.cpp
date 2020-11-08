#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/CollisionObject.h>

// arm and robot_description
const std::string PLANNING_GROUP    = "ur5_arm";
const std::string ROBOT_DESCRIPTION = "robot_description";


//https://github.com/ros-planning/moveit_tutorials/blob/master/doc/creating_moveit_plugins/lerp_motion_planner/src/lerp_example.cpp
void	
callback_ori(const gazebo_msgs::LinkStatesConstPtr& msg)
{
    ROS_INFO("Im called");
    for (size_t i = 0; i < msg->name.size(); i++)
    {
        ROS_INFO_STREAM(msg->name[i]);
    }
}

int
main(int argc, char** argv)
{
	// init node
	ros::init(argc, argv, "gazebo2rviz");
	ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // subscribe to link poses, however we still need mesh names
    const auto& sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, &callback_ori);

    // load robot model
    const auto robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

    // we we will use movegroupinterface to interface the planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // we will use planning-scene interface to add and remove collision objects in our virtual world
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // raw pointer to joint model group
    const auto joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // visualization in rviz
    moveit_visual_tools::MoveItVisualTools visual_tools("ur5_link0");

    /*
    
    // create a planning scene monitor
    pl::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    // get the model
    moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

    // create a robotstate and keep track of the current pose
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
    robot_state->setToDefaultValues();
    robot_state->update();

    // create jointmodelgroup
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string>& link_model_names = joint_model_group->getLinkModelNames();

    ROS_INFO_STREAM(link_model_names[0]);
    */

    while(ros::ok())
    {

        ROS_INFO_ONCE("hello");
        ros::Duration(1).sleep();
    }

    // https://github.com/ros-planning/moveit_tutorials/tree/kinetic-devel/doc/planning_scene

    return 0;
}