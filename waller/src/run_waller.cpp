#include "Waller.h"

int main(int argc, char **argv) {
    // Initiate new ROS node named "stopper"
    ros::init(argc, argv, "stopper");
    ROS_INFO("HELLO");
    ROS_INFO("HELLO");
    ROS_INFO("HELLO");
    ROS_INFO("HELLO");


    // Create new stopper object

    Stopper stopper;

    // Start the movement
    stopper.startMoving();

    return 0;
}
