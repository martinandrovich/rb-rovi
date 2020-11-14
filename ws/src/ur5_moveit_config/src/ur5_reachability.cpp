#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <rovi_utils/rovi_utils.h>

// arm and robot_description
const std::string PLANNING_GROUP    = "ur5_arm";
const std::string ROBOT_DESCRIPTION = "robot_description";

int
main(int argc, char** argv)
{
	using namespace rovi_utils;
	
	// init node
	ros::init(argc, argv, "reachability");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

    // create a planning scene
    ros::Publisher planning_scene_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene_2", 1);

    // remove all markers
    moveit_visual_tools::MoveItVisualTools visual_tools("ur5_link0");
    visual_tools.deleteAllMarkers();

    // load robot model
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

    // load robot kinematics
    robot_model::RobotModelPtr robot_kinematic_model = robot_model_loader->getModel();

    // we need to add a planning_scene
    planning_scene::PlanningScene planning_scene_rob(robot_kinematic_model);

    // raw pointer to joint model group
    const auto joint_model_group = planning_scene_rob.getCurrentState().getJointModelGroup(PLANNING_GROUP);

	// add collision objects
	std::vector<moveit_msgs::CollisionObject> collision_objects
	{
		make_mesh_cobj("table",  planning_scene_rob.getPlanningFrame() , {0.4, 0.6, 0.64}),
		make_mesh_cobj("bottle", planning_scene_rob.getPlanningFrame() , {0.5, 1, 0.75})
	};

    // take robot state from planning scene
    moveit_msgs::RobotState robotstate_msg;
    
    // move base
    {
        planning_scene_rob.getCurrentStateNonConst().setVariablePosition(0, 0.1);
        planning_scene_rob.getCurrentStateNonConst().setVariablePosition(1, 0.1);
        planning_scene_rob.getCurrentStateNonConst().setVariablePosition(2, 0.75);
    }

    planning_scene_rob.getCurrentStateNonConst().update();

    // planning scene
    moveit_msgs::PlanningScene ps_msg;
    planning_scene_rob.getPlanningSceneDiffMsg(ps_msg);
    ps_msg.world.collision_objects = collision_objects;
    planning_scene_rob.setPlanningSceneDiffMsg(ps_msg);

    planning_scene_pub.publish(ps_msg);

    ros::Rate lp(0.2);

    while(ros::ok())
    {

        planning_scene_rob.getCurrentStateNonConst().setJointGroupPositions(joint_model_group, {0, 1.57, 0, 0, 0, 0});
        planning_scene_rob.getPlanningSceneDiffMsg(ps_msg);
        ps_msg.world.collision_objects = collision_objects;
        planning_scene_rob.isStateColliding("ur5_arm", true);

        planning_scene_pub.publish(ps_msg);
        visual_tools.trigger();
        lp.sleep();
    }

    return 0;
}