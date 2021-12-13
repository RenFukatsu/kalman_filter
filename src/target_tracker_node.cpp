#include "kalman_filter/target_tracker.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_tracker");
    TargetTracker tracker;
    tracker.process();
    return 0;
}
