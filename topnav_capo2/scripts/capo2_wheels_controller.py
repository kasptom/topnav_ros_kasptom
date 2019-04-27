#!/usr/bin/python
import rospy

from std_msgs.msg import Float64

# from driver.dummy.dummy_wheels_driver import DummyWheelsDriver
from driver.maestro.maestro_wheels_driver import MaestroWheelsDriver

FRONT_LEFT_WHEEL_TOPIC = "/capo_front_left_wheel_controller/command"
FRONT_RIGHT_WHEEL_TOPIC = "/capo_front_right_wheel_controller/command"


# BACK_RIGHT_WHEEL_TOPIC = "/capo_rear_left_wheel_controller/command",
# BACK_LEFT_WHEEL_TOPIC = "/capo_rear_right_wheel_controller/command"


class CapoWheelsController:
    def __init__(self):
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.driver = MaestroWheelsDriver()
        # self.driver = DummyWheelsDriver()

        self.left_wheel_subscriber = rospy.Subscriber(FRONT_LEFT_WHEEL_TOPIC, Float64, queue_size=1,
                                                      callback=self.set_left_velocity)
        self.right_wheel_subscriber = rospy.Subscriber(FRONT_RIGHT_WHEEL_TOPIC, Float64, queue_size=1,
                                                       callback=self.set_right_velocity)

    def start(self):
        rospy.init_node("capo2_wheels_controller", anonymous=True)
        rospy.spin()

    def set_left_velocity(self, value):
        self.left_wheel_velocity = value.data
        self.driver.set_velocity(self.left_wheel_velocity, self.right_wheel_velocity)
        print("left vel: %.2f" % self.left_wheel_velocity)

    def set_right_velocity(self, value):
        self.right_wheel_velocity = value.data
        self.driver.set_velocity(self.left_wheel_velocity, self.right_wheel_velocity)
        print("right vel: %.2f" % self.right_wheel_velocity)


if __name__ == '__main__':
    try:
        controller = CapoWheelsController()
        controller.start()
    except rospy.ROSInterruptException:
        raise
