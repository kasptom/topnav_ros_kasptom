#ifndef TOPNAV_CONFIG_CONFIGNODE_H
#define TOPNAV_CONFIG_CONFIGNODE_H

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <topnav_config/topnavConfig.h>
#include <topnav_msgs/TopNavConfigMsg.h>

class ConfigNode {
public:
    ConfigNode();
    static void on_configuration_changed(topnav_config::topnavConfig &config, uint32_t level, ros::Publisher config_change_publisher);
private:
    ros::NodeHandle handle;
    ros::Publisher _config_change_publisher;

    static topnav_msgs::TopNavConfigMsg create_config_message(topnav_config::topnavConfig &config);

public:
    const ros::Publisher &get_config_change_publisher() const;
};


#endif //TOPNAV_CONFIG_CONFIGNODE_H
