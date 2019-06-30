#ifndef TOPNAV_SHARED_ANGLERANGE_H
#define TOPNAV_SHARED_ANGLERANGE_H


class AngleRange {
public:
    AngleRange(float angleRads, float range);

    double get_angle() const;

    double get_range() const;

    void set_range(double range);

private:
    double angle;
    double range;
};


#endif //TOPNAV_SHARED_ANGLERANGE_H
