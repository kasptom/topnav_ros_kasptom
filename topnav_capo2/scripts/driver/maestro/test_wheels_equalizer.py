from unittest import TestCase

from driver.maestro.wheels_equalizer import WheelsEqualizer, CAPO_2_WHEELS_MAP


class TestWheelsEqualizer(TestCase):

    def setUp(self):
        super(TestWheelsEqualizer, self).setUp()
        self.equalizer = WheelsEqualizer(CAPO_2_WHEELS_MAP)

    def test_equalize_right_target(self):
        equalized = self.equalizer.equalize_right_target(5100, 4960)
        self.assertEqual(6342, equalized)
