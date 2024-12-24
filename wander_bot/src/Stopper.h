#ifndef STOPPER_H
#define STOPPER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

enum class RobotStatus {
  DOWN = 1,
  TURNLEFT,
  UP,
  TURNRIGHT,

  first = DOWN,
  last = TURNRIGHT
};

class State {
  public:
    RobotStatus state;
    State();

  void nextState();
  bool toMove();
  float getTurnAngle(); 


};



class Stopper {
public:

  //Tunable parameters
  static constexpr double FORWARD_SPEED = 0.1;
  static constexpr double MIN_SCAN_ANGLE = M_PI+(-30.0/180*M_PI);
  static constexpr double MAX_SCAN_ANGLE = M_PI+(+30.0/180*M_PI);
  static constexpr float MIN_DIST_FROM_OBSTACLE = 0.4f; // Should be smaller
          // than sensor_msgs::LaserScan::range_max
  static constexpr double HORIZONTAL_DISTANCE = 0.2;

  double ROTATE_SPEED = (15* M_PI) / 180;

  Stopper();
  void startMoving();
  void rotate();
  void rotate180();
  void moveHorizontally();
  int isWallNear();


private:
  ros::NodeHandle node;
  ros::Publisher commandPub; // Publisher to the robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
  bool keepMoving; // Indicates whether the robot should continue moving
  sensor_msgs::LaserScanConstPtr laser;
  State state;
  bool isReverse;

  void moveForward();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif // STOPPER_H
