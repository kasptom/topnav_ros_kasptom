from serial import SerialException

import maestro
from constants.tty_ports import TTY_PORT_MAESTRO_DEFAULT, TTY_PORT_MAESTRO_FALLBACK_A, TTY_PORT_MAESTRO_FALLBACK_B
from driver.interface_wheels_driver import IWheelsDriver


class MaestroWheelsDriver(IWheelsDriver):
    _RIGHT_WHEEL_CHANNEL = 0
    _LEFT_WHEEL_CHANNEL = 1

    def __init__(self, left_min=4000, left_mid=6000, left_max=8000, right_min=4000, right_mid=6000, right_max=8000):
        self.right_max = right_max
        self.right_mid = right_mid
        self.right_min = right_min
        self.left_max = left_max
        self.left_mid = left_mid
        self.left_min = left_min
        self._servo = None
        self._initialize_servos()

    def set_velocity(self, left_wheel, right_wheel):
        left_wheel_target = int(left_wheel * 250 + self.left_mid)
        right_wheel_target = int(-right_wheel * 250 + self.right_mid)

        left_wheel_target = self.left_min if left_wheel_target < self.left_min else left_wheel_target
        right_wheel_target = self.right_min if right_wheel_target < self.right_min else right_wheel_target

        left_wheel_target = self.left_max if left_wheel_target > self.left_max else left_wheel_target
        right_wheel_target = self.right_max if right_wheel_target > self.right_max else right_wheel_target

        if left_wheel == 0:
            left_wheel_target = 0
        if right_wheel == 0:
            right_wheel_target = 0

        print("Setting wheel servos (%d, %d) (%d, %d)" % (MaestroWheelsDriver._LEFT_WHEEL_CHANNEL, left_wheel_target,
                                                          MaestroWheelsDriver._RIGHT_WHEEL_CHANNEL, right_wheel_target))
        self._servo.setTarget(MaestroWheelsDriver._LEFT_WHEEL_CHANNEL, left_wheel_target)
        self._servo.setTarget(MaestroWheelsDriver._RIGHT_WHEEL_CHANNEL, right_wheel_target)

    def stop_wheels(self):
        self.set_velocity(0, 0)

    def stop_driver(self):
        self.stop_wheels()
        self._servo.close()

    def _initialize_servos(self):
        try:
            self._servo = maestro.Controller(TTY_PORT_MAESTRO_DEFAULT)
        except SerialException:
            print '[wheels] could not connect to %s. Trying with %s' % (
                TTY_PORT_MAESTRO_DEFAULT, TTY_PORT_MAESTRO_FALLBACK_A)
        try:
            self._servo = self._servo if self._servo is not None else maestro.Controller(TTY_PORT_MAESTRO_FALLBACK_A)
        except SerialException:
            print '[wheels] could not connect to %s. Trying with %s' % (
                TTY_PORT_MAESTRO_FALLBACK_A, TTY_PORT_MAESTRO_FALLBACK_B)
        try:
            self._servo = self._servo if self._servo is not None else maestro.Controller(TTY_PORT_MAESTRO_FALLBACK_B)
        except SerialException:
            raise
