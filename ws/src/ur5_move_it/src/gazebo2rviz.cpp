#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>


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

    // subscribe to this dummy
    const auto& sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, &callback_ori);

    ros::Rate lp(1);

    while(ros::ok())
    {
        ros::spinOnce();
        ROS_INFO_ONCE("hello");
        lp.sleep();
    }

    return 0;
}