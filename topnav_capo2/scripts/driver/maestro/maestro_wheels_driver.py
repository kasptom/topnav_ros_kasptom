from driver.interface_wheels_driver import IWheelsDriver
from driver.maestro.wheels_equalizer import WheelsEqualizer, CAPO_2_WHEELS_MAP


class MaestroWheelsDriver(IWheelsDriver):
    _RIGHT_WHEEL_CHANNEL = 0
    _LEFT_WHEEL_CHANNEL = 1
    _WHEEL_SPEED = 60
    _WHEEL_ACCELERATION = 0
    _VELOCITY_COEFFICIENT = 125

    def __init__(self, servo,
                 left_min=4000, left_mid=6000, left_max=8000, right_min=4000, right_mid=6000, right_max=8000):
        self.right_max = right_max
        self.right_mid = right_mid
        self.right_min = right_min
        self.left_max = left_max
        self.left_mid = left_mid
        self.left_min = left_min
        self._servo = None
        self._equalizer = WheelsEqualizer(CAPO_2_WHEELS_MAP)
        self._initialize_servos(servo)

    def set_velocity(self, left_wheel, right_wheel):
        left_wheel_target = int(left_wheel * MaestroWheelsDriver._VELOCITY_COEFFICIENT + self.left_mid)
        right_wheel_target = int(-right_wheel * MaestroWheelsDriver._VELOCITY_COEFFICIENT + self.right_mid)

        left_wheel_target = self.left_min if left_wheel_target < self.left_min else left_wheel_target
        right_wheel_target = self.right_min if right_wheel_target < self.right_min else right_wheel_target

        right_wheel_target = self._equalizer.equalize_right_target(right_wheel_target, self.right_mid)

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

    def _initialize_servos(self, servo):
        self._servo = servo
        self._servo.setAccel(MaestroWheelsDriver._LEFT_WHEEL_CHANNEL, MaestroWheelsDriver._WHEEL_ACCELERATION)
        self._servo.setAccel(MaestroWheelsDriver._RIGHT_WHEEL_CHANNEL, MaestroWheelsDriver._WHEEL_ACCELERATION)

        self._servo.setSpeed(MaestroWheelsDriver._LEFT_WHEEL_CHANNEL, MaestroWheelsDriver._WHEEL_SPEED)
        self._servo.setSpeed(MaestroWheelsDriver._RIGHT_WHEEL_CHANNEL, MaestroWheelsDriver._WHEEL_SPEED)
