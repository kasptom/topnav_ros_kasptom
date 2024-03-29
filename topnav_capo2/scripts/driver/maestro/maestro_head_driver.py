from driver.interface_head_driver import IHeadDriver


class MaestroHeadDriver(IHeadDriver):

    def __init__(self, servo):
        self._LOWER_SERVO_CHANNEL = 2
        self._UPPER_SERVO_CHANNEL = 3
        self._MIN_TARGET = [2300, 2300]
        self._MAX_TARGET = [9900, 9900]
        self._MID_TARGET = [6100, 6100]
        self._SERVO_SPEED = 12
        self._MIN_ANGLE = -180
        self._MAX_ANGLE = 180
        self._MID_ANGLE = 0
        self._ANGULAR_SERVO_RANGE = 180
        self._servo = None
        self._initialize_servos(servo)

    def __del__(self):
        self.stop_driver()

    def set_head_rotation(self, angle_degrees):
        if angle_degrees > 180:
            print 'Invalid angle: %d' % angle_degrees
            angle_degrees = 180
        elif angle_degrees < -180:
            angle_degrees = -180

        upper_target = self._MID_TARGET[1]

        if -90 <= angle_degrees <= 90:
            lower_target = self._MAX_TARGET[0] - (
                    self._MAX_TARGET[0] - self._MIN_TARGET[0]) * (
                                   angle_degrees - self._MIN_ANGLE / 2) / self._ANGULAR_SERVO_RANGE
        elif angle_degrees > 90:
            lower_target = self._MIN_TARGET[0]
            upper_target = self._MID_TARGET[1] - (
                    self._MID_TARGET[1] - self._MIN_TARGET[1]) * (
                                   angle_degrees - self._MAX_ANGLE / 2) / (self._ANGULAR_SERVO_RANGE / 2)
        else:  # angle_degrees < - 90
            lower_target = self._MAX_TARGET[0]
            upper_target = self._MID_TARGET[1] + (
                    self._MAX_TARGET[1] - self._MID_TARGET[1]) * (
                                   self._MIN_ANGLE / 2 - angle_degrees) / (self._ANGULAR_SERVO_RANGE / 2)

        # print 'Setting head rotation to %f' % angle_degrees
        self._servo.setTarget(self._UPPER_SERVO_CHANNEL, int(upper_target))
        self._servo.setTarget(self._LOWER_SERVO_CHANNEL, int(lower_target))
        # print 'lower target: %d, upper target: %d' % (lower_target, upper_target)

    def get_head_rotation(self):
        lower_position = self._servo.getPosition(self._LOWER_SERVO_CHANNEL)
        upper_position = self._servo.getPosition(self._UPPER_SERVO_CHANNEL)
        # print 'lower / upper position: %.2f / %.2f' % (lower_position, upper_position)

        angle_degrees = 0

        if lower_position > self._MID_TARGET[0]:
            angle_degrees -= (self._ANGULAR_SERVO_RANGE / 2) * (
                    float(lower_position - self._MID_TARGET[0]) / (self._MAX_TARGET[0] - self._MID_TARGET[0])
            )
        else:
            angle_degrees += (self._ANGULAR_SERVO_RANGE / 2) * (
                    float(self._MID_TARGET[0] - lower_position) / (self._MID_TARGET[0] - self._MIN_TARGET[0])
            )

        if upper_position > self._MID_TARGET[1]:
            angle_degrees -= (self._ANGULAR_SERVO_RANGE / 2) * (
                    float(upper_position - self._MID_TARGET[1]) / (self._MAX_TARGET[1] - self._MID_TARGET[1])
            )
        else:
            angle_degrees += (self._ANGULAR_SERVO_RANGE / 2) * (
                    float(self._MID_TARGET[1] - upper_position) / (self._MID_TARGET[1] - self._MIN_TARGET[1])
            )

        return angle_degrees

    def reset_head_rotation(self):
        self.set_head_rotation(0)

    def stop_driver(self):
        self.reset_head_rotation()
        self._servo.close()

    def _initialize_servos(self, servo):
        self._servo = servo
        self._servo.setRange(self._LOWER_SERVO_CHANNEL,
                             self._MIN_TARGET[0],
                             self._MAX_TARGET[0])
        self._servo.setRange(self._UPPER_SERVO_CHANNEL,
                             self._MIN_TARGET[1],
                             self._MAX_TARGET[1])

        self._servo.setSpeed(self._LOWER_SERVO_CHANNEL,
                             self._SERVO_SPEED)
        self._servo.setSpeed(self._UPPER_SERVO_CHANNEL,
                             self._SERVO_SPEED)
        self.set_head_rotation(self._MID_ANGLE)
