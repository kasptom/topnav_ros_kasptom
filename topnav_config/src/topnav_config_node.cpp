#include "ConfigNode.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "topnav_config");

    dynamic_reconfigure::Server<topnav_config::topnavConfig> server;
    dynamic_reconfigure::Server<topnav_config::topnavConfig>::CallbackType f;

    ConfigNode configNode;

    f = boost::bind(ConfigNode::on_configuration_changed, _1, _2, configNode.get_config_change_publisher());
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
