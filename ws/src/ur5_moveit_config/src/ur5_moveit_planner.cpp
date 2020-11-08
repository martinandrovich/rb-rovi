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

#include <moveit_utils/moveit_utils.h>

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
    visual_tools.deleteAllMarkers();

    // init objects
    std::vector<moveit_msgs::CollisionObject> collision_objects
    {
        moveit::make_mesh_cobj("table", move_group.getPlanningFrame() , {0, 0, 0}),
        moveit::make_mesh_cobj("bottle", move_group.getPlanningFrame() , {0, 0, 0})
    };
   
    ROS_INFO_STREAM(move_group.getPlanningFrame());

    moveit::move_base("world_new", "world", {0.0, 0.0, 2.0});

    planning_scene_interface.addCollisionObjects(collision_objects);

    {
        // set a desired pose for the end-effector
        geometry_msgs::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.position.x = 0.3;
        target_pose.position.y = 0.3;
        target_pose.position.z = 0.5;

        // set a targetpose
        move_group.setPoseTarget(target_pose);

        // this is just planning, not moving the robot
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_STREAM("The plan was " << success ? "successfull" : "not successfull" );

        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' to continue RvizVisualToolsGui");
        
    }

    

    while(ros::ok())
    {

        ROS_INFO_ONCE("hello");
        ros::Duration(1).sleep();
    }

    // https://github.com/ros-planning/moveit_tutorials/tree/kinetic-devel/doc/planning_scene

    return 0;
}