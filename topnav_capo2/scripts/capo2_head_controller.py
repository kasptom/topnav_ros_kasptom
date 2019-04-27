#!/usr/bin/python
import rospy
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import JointState

from driver.dummy.dummy_head_driver import DummyHeadDriver

HEAD_JOINT_TOPIC = "/capo_head_rotation_controller/command";
CAPO_JOINT_STATES = '/joint_states'


class HeadController:

    def __init__(self):
        # self.driver = MaestroHeadDriver() todo
        self.driver = DummyHeadDriver()

        self.rotation_joint_change_subscriber = rospy.Subscriber(HEAD_JOINT_TOPIC, Float64,
                                                                 queue_size=1,
                                                                 callback=self.set_head_rotation)
        self.rotation_joint_state_publisher = rospy.Publisher(CAPO_JOINT_STATES, JointState, queue_size=1)

    def start(self):
        rospy.init_node("capo2_head_controller", anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            head_rotation = self.driver.get_head_rotation()
            message = self._create_joint_state_messsage(head_rotation)

            self.rotation_joint_state_publisher.publish(message)
            rate.sleep()

    def set_head_rotation(self, value):
        self.driver.set_head_rotation(value.data)
        # rospy.loginfo("head rotation: %.2f" % self.driver.get_head_rotation())
        print("head rotation: %.2f" % self.driver.get_head_rotation())

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


if __name__ == '__main__':
    try:
        controller = HeadController()
        controller.start()
    except rospy.ROSInterruptException:
        raise
