from abc import ABCMeta, abstractmethod


class IWheelsDriver:
    __metaclass__ = ABCMeta

    @abstractmethod
    def set_velocity(self, left_wheel, right_wheel):
        raise NotImplementedError

    @abstractmethod
    def stop_wheels(self):
        raise NotImplementedError

    @abstractmethod
    def stop_driver(self):
        raise NotImplementedError
