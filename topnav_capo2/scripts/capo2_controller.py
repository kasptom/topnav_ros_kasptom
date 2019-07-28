#!/usr/bin/python
import math
import time
from serial import SerialException
from threading import Lock

import rospy

import maestro
from capo2_head_controller import HeadController
from capo2_wheels_controller import CapoWheelsController
from constants.tty_ports import TTY_PORT_MAESTRO_DEFAULT, TTY_PORT_MAESTRO_FALLBACK_A, TTY_PORT_MAESTRO_FALLBACK_B
from driver.maestro.maestro_head_driver import MaestroHeadDriver
from driver.maestro.maestro_wheels_driver import MaestroWheelsDriver


class CapoController:
    def __init__(self):
        self.lock = Lock()
        self._servo = None
        self._prev_left_wheel_speed = None
        self._prev_right_wheel_speed = None
        self._prev_head_rotation_radians = None

        try:
            self._servo = maestro.Controller(TTY_PORT_MAESTRO_DEFAULT)
        except SerialException:
            print '[capo controller] could not connect to %s Trying with %s' \
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

        self._wheels_driver = MaestroWheelsDriver(self._servo, 5120, 5960, 6800, 5120, 5960, 6800)
        self._head_driver = MaestroHeadDriver(self._servo)

        self.head_controller = HeadController()
        self.wheels_controller = CapoWheelsController()

    def start(self):
        rospy.init_node("capo2_controller", anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        is_reading = True

        while not rospy.is_shutdown():
            if is_reading:
                try:
                    current_head_rotation_degrees = self._head_driver.get_head_rotation()
                    # print 'servos\' read head rotation: %d' % current_head_rotation_degrees
                    self.head_controller.publish_head_rotation(current_head_rotation_degrees / 180.0 * math.pi)
                except SerialException:
                    print 'could not read head rotation'
                    raise
            else:
                self.update_joints_state()
            is_reading = not is_reading
            rate.sleep()

    def update_joints_state(self):
        left_wheel_speed = self.wheels_controller.get_left_wheel_speed()
        right_wheel_speed = self.wheels_controller.get_right_wheel_speed()
        if self._prev_left_wheel_speed != left_wheel_speed or self._prev_right_wheel_speed != right_wheel_speed:
            self._prev_left_wheel_speed = left_wheel_speed
            self._prev_right_wheel_speed = right_wheel_speed
            self._wheels_driver.set_velocity(left_wheel_speed, right_wheel_speed)

        requested_head_rotation = self.head_controller.get_requested_head_rotation()
        if self._prev_head_rotation_radians != requested_head_rotation:
            self._prev_head_rotation_radians = requested_head_rotation
            self._head_driver.set_head_rotation(requested_head_rotation / math.pi * 180.0)


if __name__ == '__main__':
    try:
        print 'Starting the main node in 5.0 seconds'
        time.sleep(5.0)
        print 'starting ...'
        controller = CapoController()
        controller.start()
    except rospy.ROSInterruptException:
        raise
