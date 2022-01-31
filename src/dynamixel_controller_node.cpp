#include "kalman_filter/dynamixel_controller.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_tracker");
    DynamixelController dc;
    dc.process();
    return 0;
}
