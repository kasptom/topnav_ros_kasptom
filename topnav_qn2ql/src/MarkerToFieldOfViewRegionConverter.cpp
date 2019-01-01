#ifndef TOPNAV_QN2QL_MARKERTOFIELDOFVIEWREGIONCONVERTER_H
#define TOPNAV_QN2QL_MARKERTOFIELDOFVIEWREGIONCONVERTER_H

#include <topnav_msgs/MarkersMsg.h>
#include <topnav_msgs/TestMessage.h>
#include <constants/topic_names.h>
#include <ros/ros.h>
#include <node/PublisherSubscriberNode.h>

class MarkerToFieldOfViewRegionConverter
        : public PublisherSubscriberNode<topnav_msgs::MarkersMsg, topnav_msgs::TestMessage> {
protected:

public:
    MarkerToFieldOfViewRegionConverter() : PublisherSubscriberNode(
            TOPIC_NAME_ARUCO_DETECTION, "test/TODO/replace_test_messagek") { }

protected:
    virtual void subscription_callback(const topnav_msgs::MarkersMsg_< std::allocator<void> >::ConstPtr message) {
        ROS_INFO("detected markers count: %zu", message->markers.size());
    }
};


#endif //TOPNAV_QN2QL_MARKERTOFIELDOFVIEWREGIONCONVERTER_H
