#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "topnav_world_reset.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_INFO("Empty message received");
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe(TOPIC_NAME_TOPNAV_WORLD_RESET, 1000, chatterCallback);

    ros::spin();

    return 0;
}
