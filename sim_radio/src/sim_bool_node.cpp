// kaidi wang 
// 2023.9.12
// publish bool flag to some ros message aim to simulate radio strick
#include <ros/ros.h>
#include <std_msgs/Bool.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_bool_node");
	ros::NodeHandle nh;
    // start outer loop
    ros::Publisher control_start_pub_att = nh.advertise<std_msgs::Bool>("start_pub_att",100);
    // start inner loop
	ros::Publisher control_mode_switch_pub = nh.advertise<std_msgs::Bool>("/mode_switch",10);

    ros::Rate rate(10.0);

    std_msgs::Bool start;
    // innerloop trigger message
    std_msgs::Bool mode;
    start.data = false;
    mode.data = false;

    // input start flag
    ROS_INFO_STREAM("input Y/n, Y is start:");
    std::string str;
    std::getline(std::cin,str);
    if (str == "Y")
    {
        start.data = true;
        mode.data = true;
    }
    else
    {
        ROS_INFO_STREAM("don't start sim bool node");
    }
    while (ros::ok())
    {
        ROS_INFO_STREAM("sim bool message...");
        control_start_pub_att.publish(start);
        control_mode_switch_pub.publish(mode);
        rate.sleep();
    }

    ros::spin();

    return 0;
}