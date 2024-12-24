#include "Stopper.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

std::ostream& operator<<(std::ostream& os, const RobotStatus& status) {
    switch (status) {
        case RobotStatus::DOWN: os << "down"; break;
        case RobotStatus::TURNLEFT: os << "turn_left"; break;
        case RobotStatus::UP: os << "up"; break;
        case RobotStatus::TURNRIGHT: os << "turn_right"; break;
    }
    return os;
}


State::State() : state(RobotStatus::DOWN) {}

Stopper::Stopper()
{
    state = State();
    keepMoving = true;
    isReverse = false;
    laser = NULL;

    // Advertise a new publisher for the robot's velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscribe to the simulated robot's laser scan topic
    laserSub = node.subscribe("scan", 1, &Stopper::scanCallback, this);
}

void State::nextState() {
    if (state == RobotStatus::last) {
        state = RobotStatus::first;
    } else {
        state = static_cast<RobotStatus>(static_cast<int>(state) + 1);
    }
    std::cout << "STATE: " << state << std::endl;

    
}

bool State::toMove() {
    if ((state == RobotStatus::UP) || (state == RobotStatus::DOWN)) {
        return true;
    } else {
        return false;
    }
}

void Stopper::moveHorizontally() {
    geometry_msgs::Twist msg;
    double start = ros::Time::now().toSec();
    double end = start + ros::Duration(abs(HORIZONTAL_DISTANCE) / FORWARD_SPEED).toSec();
    while (ros::Time::now().toSec() < end) {
        msg.linear.x = -1 * FORWARD_SPEED;
        commandPub.publish(msg);
    }
    msg.linear.x = 0;
}

float State::getTurnAngle() {

    if (state == RobotStatus::TURNLEFT) {
        return (-90 * (M_PI / 180));
    } else {
        return (90 * (M_PI / 180));
    }
}

// Send a velocity command
void Stopper::moveForward() {
    geometry_msgs::Twist msg; 
    // The default constructor will set all commands to 0
    msg.linear.x = (-1)*FORWARD_SPEED;
    commandPub.publish(msg);
}



void Stopper::rotate() {
    ros::Rate rate(10);
    geometry_msgs::Twist msg;
    ROS_INFO("Rotating");
    float turnAngle = state.getTurnAngle();

    ros::Time start = ros::Time::now();
    ros::Time end = start + ros::Duration((abs(turnAngle) / ROTATE_SPEED) / 4 );
    std::cout << "END: " << end << std::endl;
    std::cout << "START: " << end << std::endl;
    while (ros::Time::now() < end) {
        int direction = 1;
        if (isReverse == true) {
            direction = -1;
        } else {
            direction = 1;
        }

        if (state.state == RobotStatus::TURNRIGHT) {
            if (isReverse == true) {
                direction = 1;
            } else {
                direction = -1;
            }
            
        }

        msg.angular.z = direction;
        commandPub.publish(msg);
        ros::spinOnce();
        rate.sleep();

    }
    msg.angular.z = 0;
    commandPub.publish(msg);
}


void Stopper::rotate180() {
    ros::Rate rate(10);
    geometry_msgs::Twist msg;
    ROS_INFO("Rotating");
    float turnAngle = (-180 * (M_PI / 180));

    if (state.state == RobotStatus::TURNLEFT) {
        turnAngle = (-180 * (M_PI / 180));
    }

    if (state.state == RobotStatus::TURNRIGHT) {
        turnAngle = (180 * (M_PI / 180));
    }


    ros::Time start = ros::Time::now();
    ros::Time end = start + ros::Duration((abs(turnAngle) / ROTATE_SPEED));
    // std::cout << "END: " << end << std::endl;
    // std::cout << "START: " << end << std::endl;
    while (ros::Time::now() < end) {
        msg.angular.z = (-1)*ROTATE_SPEED;
        commandPub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
    msg.angular.z = 0;
    commandPub.publish(msg);
    ros::spinOnce();
    rate.sleep();
}

// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    bool isObstacleInFront = false;

    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
        if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
            isObstacleInFront = true;
            break;
        }
    }

    laser = scan;

    // ROS_INFO("SCAN");


    if (isObstacleInFront && state.toMove()) {
        ROS_INFO("Obstacle! Rotating");
        state.nextState();
        isObstacleInFront = false;
        // geometry_msgs::Twist msg; // The default constructor will set all commands to 0
        // msg.linear.x = 0.0;
        // commandPub.publish(msg);
        // keepMoving = false;
    }
}

int Stopper::isWallNear() {
    bool isObstacleInFront = false;

    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - laser->angle_min) / laser->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - laser->angle_min) / laser->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
        if (laser->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
            isObstacleInFront = true;
            break;
        }
    }
    std::cout << "CHECK"<< std::endl;

    if (isObstacleInFront == true) {
            std::cout << "OBSTACLE CHECK REVERSE"<< std::endl;

        if (isReverse == true) {
            std::cout << "REVERSE WAS TRUE!!! TURN TO FALSE"<< std::endl;
            isReverse = false;
            rotate180();
        } else {
            std::cout << "TURN TO REVERSE TRUE"<< std::endl;
            isReverse = true;
            rotate180();
        }
        
    } 
        std::cout << isReverse<< std::endl;


    return 0;
}

void Stopper::startMoving()
{
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    ros::Rate rate(10);
    ROS_INFO("Start moving");

    // Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
    while (ros::ok()) {
            // ROS_INFO("ENTER");

        // bool ans = state.toMove();
        if (state.toMove() == true) {
            moveForward();
            ros::spinOnce();
            rate.sleep();
        } else {
            msg.linear.x = 0;
            rotate();
            isWallNear();
            if (isReverse == true) {


            } else {

            }
            moveHorizontally();
            rotate();
            state.nextState();
            ros::spinOnce();
            rate.sleep();
        }
        
        // moveForward();
        // ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        // rate.sleep();
    }
}
