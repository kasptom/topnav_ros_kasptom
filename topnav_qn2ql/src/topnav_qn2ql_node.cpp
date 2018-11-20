#include <ros/init.h>
#include "MarkerToFieldOfViewRegionConverter.cpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "qn2ql");
    MarkerToFieldOfViewRegionConverter converter;
    ros::spin();
    return 0;
}
