import maestro


class MaestroWheelsDriver:
    def __init__(self, left_min=4000, left_mid=6000, left_max=8000, right_min=4000, right_mid=6000, right_max=8000):
        self.right_max = right_max
        self.right_mid = right_mid
        self.right_min = right_min
        self.left_max = left_max
        self.left_mid = left_mid
        self.left_min = left_min
        self._initialize_servo()

    def set_velocity(self, left_wheel, right_wheel):
        left_wheel_target = left_wheel * 400 + self.left_mid
        right_wheel_target = right_wheel * 400 + self.right_mid

        left_wheel_target = self.left_min if left_wheel_target < self.left_min else left_wheel_target
        right_wheel_target = self.right_min if right_wheel_target < self.right_min else right_wheel_target

        left_wheel_target = self.left_max if left_wheel_target > self.left_max else left_wheel_target
        right_wheel_target = self.right_max if right_wheel_target > self.right_max else right_wheel_target

        self._servo.setTarget(0, left_wheel_target)
        self._servo.setTarget(1, right_wheel_target)

    def _initialize_servo(self):
        self._servo = maestro.Controller()

    def stop_wheels(self):
        self.set_velocity(0, 0)

    def stop_driver(self):
        self._servo.close()
