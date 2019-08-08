#!/usr/bin/python

import rospy
import time
from std_msgs.msg import UInt64

TOPIC_NAME_CAPO_CLOCK = "capo/clock"
NODE_NAME = "capo/clock"
CAPO_CLOCK_DEFAULT_RATE_HZ = 10
ROS_LAUNCH_PARAM_NAME_CLOCK_PUBLISH_RATE = "/%s/clock_publish_rate_hz" % NODE_NAME


class Capo2Clock:
    def __init__(self):
        self.publisher = rospy.Publisher(TOPIC_NAME_CAPO_CLOCK, UInt64, queue_size=1)
        self.publish_rate = rospy.get_param(ROS_LAUNCH_PARAM_NAME_CLOCK_PUBLISH_RATE, CAPO_CLOCK_DEFAULT_RATE_HZ)
        print 'publishing clock with %d [Hz] rate' % self.publish_rate

    def start(self):
        rospy.init_node("capo/clock/publisher", anonymous=True)
        rate = rospy.Rate(CAPO_CLOCK_DEFAULT_RATE_HZ)

        while not rospy.is_shutdown():
            self.publisher.publish(time.time())
            rate.sleep()


if __name__ == '__main__':
    try:
        publisher = Capo2Clock()
        publisher.start()
    except rospy.ROSInterruptException:
        print 'clock interrupted - stopping...'
