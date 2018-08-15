#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "topnav_world_reset.h"
#include "gazebo_msgs/SetModelState.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void laserScanCallback(const std_msgs::Empty::ConstPtr &msg)
{
    ROS_INFO("Empty message received");

    ros::NodeHandle n;
    gazebo_msgs::ModelState modelState;
    gazebo_msgs::SetModelState setModelState;

    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    modelState.model_name = "capo";
    setModelState.request.model_state = modelState;

    if(client.call(setModelState)) {
        ROS_INFO("Successfully reset model");
    } else {
        ROS_ERROR("Could not reset the model");
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe(TOPIC_NAME_TOPNAV_WORLD_RESET, 1000, laserScanCallback);

    ros::spin();

    return 0;
}
