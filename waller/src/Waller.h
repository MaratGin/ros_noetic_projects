#ifndef STOPPER_H
#define STOPPER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

struct Values {
  public:
    int isObstacleInFront;
    float avg;
    Values();
};

enum class RobotState {
  FORWARD = 1,
  TURN_LEFT,
  FORWARD_LEFT,
  FIND_WALL,

  first = FORWARD,
  last = FIND_WALL
};

class Sensor {
  public:
    double left;
    double right;
    Sensor(float angle, float spread);
    Sensor();
    static constexpr float MIN_DIST_FROM_OBSTACLE = 0.4f; // Should be smaller

  
  Values isFree(const sensor_msgs::LaserScan::ConstPtr& scan) const;
  void isFree();
  double toRad();
};



class RobotData {
  public:
    RobotState state;
    Sensor right;
    Sensor front;
    RobotData();

  void nextState();
  float getTurnAngle(); 
};



class Stopper {
public:

  //Tunable parameters
  static constexpr double FORWARD_SPEED = 0.1;
  static constexpr double MIN_SCAN_ANGLE = M_PI+(-15.0/180*M_PI);
  static constexpr double MAX_SCAN_ANGLE = M_PI+(+15.0/180*M_PI);
  static constexpr float MIN_DIST_FROM_OBSTACLE = 0.4f; // Should be smaller
          // than sensor_msgs::LaserScan::range_max
  static constexpr double HORIZONTAL_DISTANCE = 0.2;

  double ROTATE_SPEED = (30* M_PI) / 180;

  Stopper();
  void startMoving();
  void rotate();
  void moveHorizontally();
  void moveDiagonal();
  void moveForward();

private:
  ros::NodeHandle node;
  ros::Publisher commandPub; // Publisher to the robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
  bool keepMoving; // Indicates whether the robot should continue moving
  RobotData data;
  double rightAvg;
  double frontAvg;
  double front;
  double right;



  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif // STOPPER_H
