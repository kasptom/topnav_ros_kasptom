#include "LaserParameters.h"

LaserParameters::LaserParameters(float angle_min, float angle_max, float range_min, float range_max, int beam_count) {
    this->angle_min = angle_min;
    this->angle_max = angle_max;
    this->range_min = range_min;
    this->range_max = range_max;
    this->beam_count = beam_count;
    this->angle_step = (angle_max - angle_min)/beam_count;
    this->range_step = (range_max - range_min)/beam_count;
}
