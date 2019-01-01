#include "AngleRange.h"

AngleRange::AngleRange(float angleRads, float range) {
    this->angle = angleRads;
    this->range = range;
}

double AngleRange::get_angle() const {
    return angle;
}

double AngleRange::get_range() const {
    return range;
}
