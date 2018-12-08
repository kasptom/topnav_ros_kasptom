#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <topnav_config/topnavConfig.h>

void callback(topnav_config::topnavConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d", config.line_detection_threshold);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "topnav_config");

    dynamic_reconfigure::Server<topnav_config::topnavConfig> server;
    dynamic_reconfigure::Server<topnav_config::topnavConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}