#include <iostream>
#include <robot_state_publisher/robot_state_publisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <ur5_kin.hpp>

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "joint_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    ros::Publisher joint_state = nh.advertise<std_msgs::Float64MultiArray>("/ur5_joint_group_pos_controller/command", 1);

    loop_rate.sleep();

    while(true)
    {
        std_msgs::Float64MultiArray arr;
        // The Arm
        arr.data.push_back(0);
        arr.data.push_back(-1.5);
        arr.data.push_back(0);
        arr.data.push_back(0);
        arr.data.push_back(0);
        arr.data.push_back(0);

        // The WSG
        arr.data.push_back(0);
        arr.data.push_back(0);
        
        joint_state.publish(arr);
        loop_rate.sleep();
    }

    ROS_INFO("i have published the target position");

    return 0;
}