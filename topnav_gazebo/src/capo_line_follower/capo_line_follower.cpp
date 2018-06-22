#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include <sstream>
#include <gazebo_msgs/SetModelState.h>

void chatterCallback(const std_msgs::Empty::ConstPtr& msg)
{
    //TODO logika do publishowania
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


    ros::Publisher reset_pub = n.advertise<std_msgs::Empty>("tutej topic do sterowania kolami", 1000);
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
    //-------- trza przeniesc ---------------
}

int main(int argc, char **argv) {

    //---------mienso subscribera---------
    ros::init(argc, argv, "laser_date_reader");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/capo/laser/scan", 1000, chatterCallback);

    ros::spin();
    //------------------


    return 0;
}
