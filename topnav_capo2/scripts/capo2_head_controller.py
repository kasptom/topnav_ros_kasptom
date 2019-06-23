#!/usr/bin/python
import math

import rospy
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import JointState

from constants.topic_names import HEAD_JOINT_TOPIC, CAPO_JOINT_STATES
from driver.maestro.maestro_head_driver import MaestroHeadDriver


class HeadController:

    def __init__(self, servo, lock):
        self.driver = MaestroHeadDriver(servo)
        self.lock = lock

        self.rotation_joint_change_subscriber = rospy.Subscriber(HEAD_JOINT_TOPIC, Float64,
                                                                 queue_size=1,
                                                                 callback=self.set_head_rotation)
        self.rotation_joint_state_publisher = rospy.Publisher(CAPO_JOINT_STATES, JointState, queue_size=1)

    def set_head_rotation(self, value):
        self.driver.set_head_rotation(value.data * 180 / math.pi)
        # rospy.loginfo("head rotation: %.2f" % self.driver.get_head_rotation())
        # print "head rotation: %.2f" % self.driver.get_head_rotation()

    def publish_head_rotation(self):
        self.lock.acquire()
        head_rotation = self.driver.get_head_rotation()
        self.lock.release()
        # print "publish head rotation: %.2f" % head_rotation
        message = self._create_joint_state_messsage(head_rotation / 180.0 * math.pi)
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
