/*
 * hello.cpp
 *
 *  Created on: Sep 15, 2017
 *      Author: lirs
 */

#include <iostream>
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Hello");

    ros::NodeHandle nh;
    ros::Rate loop_rate(3);

    int count = 0;
    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
        ROS_INFO_STREAM("hello world ! " << count);

        ros::spinOnce(); // Allow ROS to process incoming messages
        loop_rate.sleep(); // Sleep for the rest of the cycle
        count++;
    }
    return 0;
}
