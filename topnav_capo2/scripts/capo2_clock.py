#!/usr/bin/python

import rospy
import time
from std_msgs.msg import Int64

TOPIC_NAME_CAPO2_CLOCK = "capo2/clock"
CAPO2_CLOCK_DEFAULT_RATE_HZ = 10


class Capo2Clock:
    def __init__(self):
        self.publisher = rospy.Publisher(TOPIC_NAME_CAPO2_CLOCK, Int64, queue_size=1)

    def start(self):
        rospy.init_node("capo2_clock_publisher", anonymous=True)
        rate = rospy.Rate(CAPO2_CLOCK_DEFAULT_RATE_HZ)

        while not rospy.is_shutdown():
            self.publisher.publish(time.time())
            rate.sleep()


if __name__ == '__main__':
    try:
        publisher = Capo2Clock()
        publisher.start()
    except rospy.ROSInterruptException:
        raise
