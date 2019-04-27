from abc import ABCMeta, abstractmethod


class IHeadDriver:
    __metaclass__ = ABCMeta

    @abstractmethod
    def set_head_rotation(self, angle_degrees):
        raise NotImplementedError

    @abstractmethod
    def get_head_rotation(self):
        raise NotImplementedError

    @abstractmethod
    def reset_head_rotation(self):
        raise NotImplementedError

    @abstractmethod
    def stop_driver(self):
        raise NotImplementedError
