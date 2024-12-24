#include "Stopper.h"

int main(int argc, char **argv) {
    // Initiate new ROS node named "stopper"
    ros::init(argc, argv, "stopper");
    ROS_INFO("STAAAAAAAAAAAAAAAAAAAAAAART");
    // Create new stopper object
    State state;
    Stopper stopper;

    // Start the movement
    stopper.startMoving();

    return 0;
}
