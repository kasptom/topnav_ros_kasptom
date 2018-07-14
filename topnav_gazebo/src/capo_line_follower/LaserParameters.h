#ifndef TOPNAV_GAZEBO_LASERPARAMETERS_H
#define TOPNAV_GAZEBO_LASERPARAMETERS_H

static const int RANGE_STEPS = 10;

class LaserParameters {
public:
    LaserParameters(float angle_min, float angle_max, float range_min, float range_max, long beam_count);
    float get_angle_min();
    float get_angle_max();
    float get_range_min();
    float get_range_max();
    long get_beam_count();
    float get_angle_step();
    float get_range_step();

private:

    float angle_min;
    float angle_max;
    float range_min;
    float range_max;
    long beam_count;
    float angle_step;
    float range_step;


};


#endif //TOPNAV_GAZEBO_LASERPARAMETERS_H
