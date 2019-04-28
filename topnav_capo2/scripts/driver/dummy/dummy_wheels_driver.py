from driver.interface_wheels_driver import IWheelsDriver


class DummyWheelsDriver(IWheelsDriver):
    def set_velocity(self, left_wheel, right_wheel):
        print('setting velocity to %.2f, %.2f' % (left_wheel, right_wheel))

    def stop_wheels(self):
        self.set_velocity(0, 0)

    def stop_driver(self):
        pass
