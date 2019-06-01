from driver.interface_head_driver import IHeadDriver
from driver.maestro import maestro


class MaestroHeadDriver(IHeadDriver):

    def __init__(self):
        self._MIN_TARGET = 2000
        self._MAX_TARGET = 10000
        self._LOWER_SERVO_CHANNEL = 2
        self._UPPER_SERVO_CHANNEL = 3
        self._SERVO_SPEED = 12
        self._MIN_ANGLE = -180
        self._MAX_ANGLE = 180
        self._MID_ANGLE = 0
        self._ANGULAR_SERVO_RANGE = 180
        self._initialize_servos()

    def set_head_rotation(self, angle_degrees):
        if angle_degrees > 180 or angle_degrees < -180:
            print('Invalid angle: %d', angle_degrees)
            return

        self._servo.setTarget(self._LOWER_SERVO_CHANNEL,
                              self._MIN_TARGET + (self._MAX_TARGET - self._MIN_TARGET) * (
                                      min(angle_degrees, 0) - self._MIN_TARGET) / self._ANGULAR_SERVO_RANGE)

        self._servo.setTarget(self._UPPER_SERVO_CHANNEL,
                              self._MIN_TARGET + (self._MAX_TARGET - self._MIN_TARGET) * (
                                      max(angle_degrees, 0) - self._MIN_TARGET) / self._ANGULAR_SERVO_RANGE)

    def get_head_rotation(self):
        rotation = self._MIN_ANGLE
        left_position = self._servo.getPosition(self._LOWER_SERVO_CHANNEL)
        rotation += self._ANGULAR_SERVO_RANGE * (left_position - self._MIN_TARGET) / (
                    self._MAX_TARGET - self._MIN_TARGET)

        right_position = self._servo.getPosition(self._UPPER_SERVO_CHANNEL)
        rotation += self._ANGULAR_SERVO_RANGE * (right_position - self._MIN_TARGET) / (
                    self._MAX_TARGET - self._MIN_TARGET)

    def reset_head_rotation(self):
        self._servo.setTarget(self._LOWER_SERVO_CHANNEL,
                              (self._MIN_TARGET + self._MAX_TARGET) / 2)
        self._servo.setTarget(self._UPPER_SERVO_CHANNEL,
                              (self._MIN_TARGET + self._MAX_TARGET) / 2)

    def stop_driver(self):
        self.reset_head_rotation()
        self._servo.close()

    def _initialize_servos(self):
        self._servo = maestro.Controller()
        self._servo.setRange(self._LOWER_SERVO_CHANNEL,
                             self._MIN_TARGET,
                             self._MAX_TARGET)
        self._servo.setRange(self._UPPER_SERVO_CHANNEL,
                             self._MIN_TARGET,
                             self._MAX_TARGET)

        self._servo.setSpeed(self._LOWER_SERVO_CHANNEL,
                             self._SERVO_SPEED)
        self._servo.setSpeed(self._UPPER_SERVO_CHANNEL,
                             self._SERVO_SPEED)
        self.set_head_rotation(self._MID_ANGLE)
