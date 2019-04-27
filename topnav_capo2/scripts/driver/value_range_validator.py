class ValueRangeValidator:
    def __init__(self, min_value, max_value):
        self.min = min_value
        self.max = max_value

    def validate(self, value):
        if self.min <= value <= self.max:
            return
        else:
            raise ValueRangeException


class ValueRangeException(Exception):
    def __init__(self):
        super(self).__init__(self)
