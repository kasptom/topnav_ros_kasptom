#include "LaserParameters.h"

LaserParameters::LaserParameters(float angle_min, float angle_max, float range_min, float range_max, long beam_count) {
    this->angle_min = angle_min;
    this->angle_max = angle_max;
    this->range_min = range_min;
    this->range_max = range_max;
    this->beam_count = beam_count;
    this->angle_step = (angle_max - angle_min) / beam_count;
    this->range_step = (range_max - range_min) / beam_count;
}

float LaserParameters::get_angle_min() {
    return this->angle_min;
}

float LaserParameters::get_angle_max() {
    return this->angle_max;
}

float LaserParameters::get_range_min() {
    return this->range_min;
}

float LaserParameters::get_range_max() {
    return this->range_max;
}

long LaserParameters::get_beam_count() {
    return this->beam_count;
}

float LaserParameters::get_angle_step() {
    return this->angle_step;
}

float LaserParameters::get_range_step() {
    return this->range_step;
}
