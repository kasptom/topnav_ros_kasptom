LEFT_WHEEL_SERVO_MAP_IDX = 0
RIGHT_WHEEL_SERVO_AMP_IDX = 1

DEFAULT_WHEELS_MAP = [(900, 2100), (1500, 1500), (2100, 900)]

CAPO_2_WHEELS_MAP = [(900, 2100),
                     (1200, 1795),
                     (1410, 1585.5),
                     (1420, 1573.5),
                     (1430, 1566),
                     (1490, 1490),
                     (1540, 1456.75),
                     (1550, 1438),
                     (1575, 1420),
                     (1780, 1230),
                     (2100, 900)]


class WheelsEqualizer:

    def __init__(self, targets_map=DEFAULT_WHEELS_MAP):
        self.targets_map = self.rescale_for_maestro(targets_map)

    def equalize_right_target(self, initial_target, mid_target):
        if initial_target == 0:
            return initial_target
        symmetric_left_target = mid_target + (mid_target - initial_target)
        left_lower_bound = self.find_left_lower_bound(symmetric_left_target)
        left_upper_bound = self.find_left_upper_bound(symmetric_left_target)
        right_lower_bound = left_upper_bound[RIGHT_WHEEL_SERVO_AMP_IDX]
        right_upper_bound = left_lower_bound[RIGHT_WHEEL_SERVO_AMP_IDX]

        right_equalized_target = int(right_lower_bound
                                     + (right_upper_bound - right_lower_bound)
                                     * (1 - (symmetric_left_target - left_lower_bound[LEFT_WHEEL_SERVO_MAP_IDX])
                                        / float(left_upper_bound[LEFT_WHEEL_SERVO_MAP_IDX]
                                                - left_lower_bound[LEFT_WHEEL_SERVO_MAP_IDX])
                                        ))
        assert right_lower_bound <= right_equalized_target <= right_upper_bound

        return right_equalized_target

    @staticmethod
    def rescale_for_maestro(targets_map):
        return [(int(target[LEFT_WHEEL_SERVO_MAP_IDX] * 4), int(target[RIGHT_WHEEL_SERVO_AMP_IDX] * 4))
                for target in targets_map]

    def find_left_lower_bound(self, symmetric_left_target):
        return max(filter(lambda target_pair: target_pair[LEFT_WHEEL_SERVO_MAP_IDX] < symmetric_left_target,
                          self.targets_map))

    def find_left_upper_bound(self, symmetric_left_target):
        return min(filter(lambda target_pair: target_pair[LEFT_WHEEL_SERVO_MAP_IDX] >= symmetric_left_target,
                          self.targets_map))


if __name__ == '__main__':
    equalizer = WheelsEqualizer(CAPO_2_WHEELS_MAP)
    for target in [4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000]:
        print (equalizer.equalize_right_target(target, 5960))
