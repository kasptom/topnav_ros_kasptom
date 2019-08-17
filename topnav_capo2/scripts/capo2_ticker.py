#!/usr/bin/python

import rospy
import time
from std_msgs.msg import UInt64

from constants.parameter_names import CAPO_TICKER_DEFAULT_RATE_HZ, TOPIC_NAME_CAPO_TICKER, \
    ROS_LAUNCH_PARAM_NAME_TICKER_PUBLISH_RATE


class Capo2Ticker:
    def __init__(self):
        self.publisher = rospy.Publisher(TOPIC_NAME_CAPO_TICKER, UInt64, queue_size=1)
        self.publish_rate = rospy.get_param(ROS_LAUNCH_PARAM_NAME_TICKER_PUBLISH_RATE, CAPO_TICKER_DEFAULT_RATE_HZ)
        print 'publishing ticker with %d [Hz] rate' % self.publish_rate

    def start(self):
        rospy.init_node("ticker_publisher", anonymous=True)
        rate = rospy.Rate(CAPO_TICKER_DEFAULT_RATE_HZ)

        while not rospy.is_shutdown():
            self.publisher.publish(time.time())
            rate.sleep()


if __name__ == '__main__':
    try:
        publisher = Capo2Ticker()
        publisher.start()
    except rospy.ROSInterruptException:
        print 'ticker interrupted - stopping...'
