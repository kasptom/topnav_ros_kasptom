#ifndef TOPNAV_GAZEBO_LASERPARAMETERS_H
#define TOPNAV_GAZEBO_LASERPARAMETERS_H


class LaserParameters {
public:
    LaserParameters(float angle_min, float angle_max, float range_min, float range_max, int beam_count);

private:

    float angle_min;
    float angle_max;
    float range_min;
    float range_max;
    int beam_count;
    float angle_step;
    float range_step;
};


#endif //TOPNAV_GAZEBO_LASERPARAMETERS_H
