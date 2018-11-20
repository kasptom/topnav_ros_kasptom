#ifndef TOPNAV_QN2QL_MARKERTOFIELDOFVIEWREGIONCONVERTER_H
#define TOPNAV_QN2QL_MARKERTOFIELDOFVIEWREGIONCONVERTER_H

#include <topnav_msgs/Markers.h>
#include <topnav_msgs/TestMessage.h>
#include <constants/topic_names.h>
#include <ros/ros.h>
#include <node/PublisherSubscriberNode.h>

class MarkerToFieldOfViewRegionConverter
        : public PublisherSubscriberNode<topnav_msgs::Markers, topnav_msgs::TestMessage> {
protected:
    std::string get_published_topic_name()
    {
        return "test/TODO/replace_test_messagek";
    }

    std::string get_subscribed_topic_name() {
        return TOPIC_NAME_ARUCO_DETECTION;
    }

    virtual void subscription_callback(const topnav_msgs::Markers_< std::allocator<void> >::ConstPtr message) {
        ROS_INFO("received message");
    }
};


#endif //TOPNAV_QN2QL_MARKERTOFIELDOFVIEWREGIONCONVERTER_H
