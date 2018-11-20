#ifndef TOPNAV_QN2QL_PUBLISHERSUBSCRIBERNODE_H
#define TOPNAV_QN2QL_PUBLISHERSUBSCRIBERNODE_H

#include <iostream>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>

template<class S, class P>
class PublisherSubscriberNode {

public:
    PublisherSubscriberNode();

protected:
    ros::NodeHandle handle;
    ros::Publisher publisher;
    ros::Subscriber subscriber;

    virtual std::string get_published_topic_name()=0;

    virtual std::string get_subscribed_topic_name()=0;

    virtual void subscription_callback(const typename S::ConstPtr message)= 0;

private:
    void initPublisherAndSubscriber();
};

template<class S, class P>
PublisherSubscriberNode<S, P>::PublisherSubscriberNode() {
    initPublisherAndSubscriber();
}

template<class S, class P>
void PublisherSubscriberNode<S, P>::initPublisherAndSubscriber() {
    subscriber = handle.subscribe(get_subscribed_topic_name(), 1000, &PublisherSubscriberNode::subscription_callback,
                                  this);
    publisher = handle.advertise<P>(get_published_topic_name(), 1000);
}


#endif //TOPNAV_QN2QL_PUBLISHERSUBSCRIBERNODE_H
