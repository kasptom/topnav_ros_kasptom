#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include <sstream>
#include "topnav_world_reset.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher reset_pub = n.advertise<std_msgs::Empty>(TOPIC_NAME_TOPNAV_WORLD_RESET, 1000);
    ros::Rate loop_rate(10);


    int count = 0;
    while (ros::ok()) {
        ROS_INFO("Press enter to send the reset signal");

        getchar();

        std_msgs::Empty msg;

        ROS_INFO("Sending an empty message...");

        reset_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
