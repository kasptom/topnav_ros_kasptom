LEFT_WHEEL_SERVO_MAP_IDX = 0
RIGHT_WHEEL_SERVO_AMP_IDX = 1

DEFAULT_WHEELS_MAP = [(1000, 1000), (1500, 1500), (2000, 2000)]

CAPO_2_WHEELS_MAP = [(1200, 1795),
                     (1410, 1585.5),
                     (1420, 1573.5),
                     (1430, 1566),
                     (1490, 1490),
                     (1540, 1456.75),
                     (1550, 1438),
                     (1575, 1420),
                     (1780, 1230)]


class WheelsEqualizer:

    def __init__(self, targets_map=DEFAULT_WHEELS_MAP):
        self.targets_map = self.rescale_for_maestro(targets_map)

    def equalize_right_target(self, initial_target, mid_target):
        symmetric_left_target = mid_target + (mid_target - initial_target)
        left_lower_bound = self.find_left_lower_bound(symmetric_left_target)
        left_upper_bound = self.find_left_upper_bound(symmetric_left_target)
        right_lower_bound = left_upper_bound[RIGHT_WHEEL_SERVO_AMP_IDX]
        right_upper_bound = left_lower_bound[RIGHT_WHEEL_SERVO_AMP_IDX]

        right_equalized_target = right_lower_bound + (right_upper_bound - right_lower_bound) * (
                (symmetric_left_target - left_lower_bound[LEFT_WHEEL_SERVO_MAP_IDX]) / (
                    left_upper_bound[LEFT_WHEEL_SERVO_MAP_IDX] - left_lower_bound[LEFT_WHEEL_SERVO_MAP_IDX]))
        return right_equalized_target

    @staticmethod
    def rescale_for_maestro(targets_map):
        return [(int(target[LEFT_WHEEL_SERVO_MAP_IDX] * 4), int(target[RIGHT_WHEEL_SERVO_AMP_IDX] * 4))
                for target in targets_map]

    def find_left_lower_bound(self, symmetric_left_target):
        return max(filter(lambda target_pair: target_pair[LEFT_WHEEL_SERVO_MAP_IDX] <= symmetric_left_target,
                          self.targets_map))

    def find_left_upper_bound(self, symmetric_left_target):
        return min(filter(lambda target_pair: target_pair[LEFT_WHEEL_SERVO_MAP_IDX] >= symmetric_left_target,
                          self.targets_map))
