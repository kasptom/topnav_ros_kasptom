from abc import ABCMeta, abstractmethod


class ICamera:
    __metaclass__ = ABCMeta

    @abstractmethod
    def open(self):
        raise NotImplementedError

    @abstractmethod
    def is_opened(self):
        raise NotImplementedError

    @abstractmethod
    def close(self):
        raise NotImplementedError

    @abstractmethod
    def get_frame(self):
        raise NotImplementedError
