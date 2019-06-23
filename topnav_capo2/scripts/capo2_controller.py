#!/usr/bin/python
from serial import SerialException

import rospy
import maestro

from capo2_head_controller import HeadController
from capo2_wheels_controller import CapoWheelsController
from constants.tty_ports import TTY_PORT_MAESTRO_DEFAULT, TTY_PORT_MAESTRO_FALLBACK_A, TTY_PORT_MAESTRO_FALLBACK_B


class CapoController:
    def __init__(self):
        try:
            self._servo = maestro.Controller(TTY_PORT_MAESTRO_DEFAULT)
        except SerialException:
            print '[capo controller] could not connect to %s Trying with %s'\
                  % (TTY_PORT_MAESTRO_DEFAULT, TTY_PORT_MAESTRO_FALLBACK_A)
        try:
            self._servo = self._servo if self._servo is not None else maestro.Controller(TTY_PORT_MAESTRO_FALLBACK_A)
        except SerialException:
            print '[capo controller] could not connect to %s Trying with %s' \
                  % (TTY_PORT_MAESTRO_FALLBACK_A, TTY_PORT_MAESTRO_FALLBACK_B)
        try:
            self._servo = self._servo if self._servo is not None else maestro.Controller(TTY_PORT_MAESTRO_FALLBACK_B)
        except SerialException:
            raise

        self.head_controller = HeadController(self._servo)
        self.wheels_controller = CapoWheelsController(self._servo)

    def start(self):
        rospy.init_node("capo2_controller", anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            self.head_controller.publish_head_rotation()
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = CapoController()
        controller.start()
    except rospy.ROSInterruptException:
        raise
