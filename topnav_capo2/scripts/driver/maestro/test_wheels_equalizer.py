from unittest import TestCase

from driver.maestro.wheels_equalizer import WheelsEqualizer, CAPO_2_WHEELS_MAP


class TestWheelsEqualizer(TestCase):

    def setUp(self):
        super(TestWheelsEqualizer, self).setUp()
        self.equalizer = WheelsEqualizer(CAPO_2_WHEELS_MAP)

    def test_equalize_right_target(self):
        equalized_1 = self.equalizer.equalize_right_target(4000, 5960)
        equalized_2 = self.equalizer.equalize_right_target(4500, 5960)
        equalized_3 = self.equalizer.equalize_right_target(5000, 5960)
        equalized_4 = self.equalizer.equalize_right_target(5500, 5960)
        equalized_5 = self.equalizer.equalize_right_target(6000, 5960)
        equalized_6 = self.equalizer.equalize_right_target(6500, 5960)
        equalized_7 = self.equalizer.equalize_right_target(7000, 5960)
        equalized_8 = self.equalizer.equalize_right_target(7500, 5960)
        equalized_9 = self.equalizer.equalize_right_target(8000, 5960)

        self.assertEqual(4095, equalized_1)
        self.assertEqual(4610, equalized_2)
        self.assertEqual(5105, equalized_3)
        self.assertEqual(5568, equalized_4)
        self.assertEqual(6010, equalized_5)
        self.assertEqual(6561, equalized_6)
        self.assertEqual(7060, equalized_7)
        self.assertEqual(7566, equalized_8)
        self.assertEqual(8074, equalized_9)

    def test_equalize_handle_zero_target(self):
        equalized = self.equalizer.equalize_right_target(0, 5960)
        self.assertEqual(0, equalized)
