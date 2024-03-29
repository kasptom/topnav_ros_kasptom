#include "ConfigNode.h"
#include <constants/topic_names.h>

ConfigNode::ConfigNode() {
    _config_change_publisher = handle.advertise<topnav_msgs::TopNavConfigMsg>(TOPIC_NAME_TOPNAV_CONFIG, 1000);
}

void ConfigNode::on_configuration_changed(topnav_config::topnavConfig &config, uint32_t level, ros::Publisher config_change_publisher) {
    ROS_INFO("Reconfigure Request: %d", config.line_detection_threshold);
    topnav_msgs::TopNavConfigMsg message = create_config_message(config);
    config_change_publisher.publish(message);
}

topnav_msgs::TopNavConfigMsg ConfigNode::create_config_message(topnav_config::topnavConfig &config) {
    topnav_msgs::TopNavConfigMsg message;
    message.line_detection_threshold = config.line_detection_threshold;
    message.prop_coeff_angle = config.pd_Kp_ang;
    message.deriv_coeff_angle = config.pd_Kd_ang;
    message.prop_coeff_dist = config.pd_Kp_dst;
    message.deriv_coeff_dist = config.pd_Kd_dst;
    message.hough_max_point_range = config.hough_max_point_range;
    return message;
}

const ros::Publisher &ConfigNode::get_config_change_publisher() const {
    return _config_change_publisher;
}
