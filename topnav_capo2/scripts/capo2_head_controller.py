#!/usr/bin/python
import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Header

from constants.topic_names import HEAD_JOINT_TOPIC, CAPO_JOINT_STATES


class HeadController:

    def __init__(self):
        self._requested_head_rotation_radians = None
        self.rotation_joint_change_subscriber = rospy.Subscriber(HEAD_JOINT_TOPIC, Float64,
                                                                 queue_size=1,
                                                                 callback=self.set_head_rotation)
        self.rotation_joint_state_publisher = rospy.Publisher(CAPO_JOINT_STATES, JointState, queue_size=1)

    def set_head_rotation(self, message_rotation_degrees):
        self._requested_head_rotation_radians = message_rotation_degrees.data * 180 / math.pi
        # rospy.loginfo("head rotation: %.2f" % self.driver.get_head_rotation())
        # print "head rotation: %.2f" % self.driver.get_head_rotation()

    def get_requested_head_rotation(self):
        return self._requested_head_rotation_radians

    def publish_head_rotation(self, rotation_radians):
        # print "publish head rotation: %.2f" % head_rotation
        message = self._create_joint_state_messsage(rotation_radians)
        self.rotation_joint_state_publisher.publish(message)

    @staticmethod
    def _create_joint_state_messsage(head_rotation):
        # print('publishing head rotation %.2f' % head_rotation)
        message = JointState()
        message.header = Header()
        message.header.stamp = rospy.Time.now()
        message.name = ['head_swivel']
        message.position = [head_rotation]
        message.velocity = []
        message.effort = []
        return message
