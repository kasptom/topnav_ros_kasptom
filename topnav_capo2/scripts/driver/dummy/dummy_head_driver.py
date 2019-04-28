from driver.interface_head_driver import IHeadDriver
from driver.value_range_validator import ValueRangeValidator


class DummyHeadDriver(IHeadDriver):

    def __init__(self):
        self._head_rotation_degrees = 0.0
        self._range_validator = ValueRangeValidator(-180.0, 180.0)

    def get_head_rotation(self):
        return self._head_rotation_degrees

    def set_head_rotation(self, angle_degrees):
        self._range_validator.validate(angle_degrees)
        self._head_rotation_degrees = angle_degrees

    def reset_head_rotation(self):
        self.set_head_rotation(0.0)

    def stop_driver(self):
        self.set_head_rotation(0.0)
        pass
