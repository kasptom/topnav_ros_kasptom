#ifndef TOPNAV_QN2QL_PUBLISHERSUBSCRIBERNODE_H
#define TOPNAV_QN2QL_PUBLISHERSUBSCRIBERNODE_H

#include <iostream>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>

template<class S, class P>
class PublisherSubscriberNode {

public:
    PublisherSubscriberNode(const std::string &subscribed_topic_name, const std::string &published_topic_name);

protected:
    ros::NodeHandle handle;
    ros::Publisher publisher;
    ros::Subscriber subscriber;

    virtual void subscription_callback(typename S::ConstPtr message)= 0;

private:
    void initPublisherAndSubscriber(const std::string &subscribed_topic_name, const std::string &published_topic_name);
};

template<class S, class P>
PublisherSubscriberNode<S, P>::PublisherSubscriberNode(const std::string &subscribed_topic_name, const std::string &published_topic_name) {
    initPublisherAndSubscriber(subscribed_topic_name, published_topic_name);
}

template<class S, class P>
void PublisherSubscriberNode<S, P>::initPublisherAndSubscriber(const std::string &subscribed_topic_name, const std::string &published_topic_name) {
    subscriber = handle.subscribe(subscribed_topic_name, 1000, &PublisherSubscriberNode::subscription_callback,
                                  this);
    publisher = handle.advertise<P>(published_topic_name, 1000);
}


#endif //TOPNAV_QN2QL_PUBLISHERSUBSCRIBERNODE_H
