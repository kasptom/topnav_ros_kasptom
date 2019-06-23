#!/usr/bin/python

import rospy
from std_msgs.msg import Float64

# from driver.dummy.dummy_wheels_driver import DummyWheelsDriver
from constants.topic_names import FRONT_LEFT_WHEEL_TOPIC, FRONT_RIGHT_WHEEL_TOPIC


class CapoWheelsController:
    def __init__(self):
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.left_wheel_subscriber = rospy.Subscriber(FRONT_LEFT_WHEEL_TOPIC, Float64, queue_size=1,
                                                      callback=self.set_left_velocity)
        self.right_wheel_subscriber = rospy.Subscriber(FRONT_RIGHT_WHEEL_TOPIC, Float64, queue_size=1,
                                                       callback=self.set_right_velocity)

    def set_left_velocity(self, value):
        self.left_wheel_velocity = value.data
        # print("left vel: %.2f" % self.left_wheel_velocity)

    def set_right_velocity(self, value):
        self.right_wheel_velocity = value.data
        # print("right vel: %.2f" % self.right_wheel_velocity)

    def get_left_wheel_speed(self):
        return self.left_wheel_velocity

    def get_right_wheel_speed(self):
        return self.right_wheel_velocity
