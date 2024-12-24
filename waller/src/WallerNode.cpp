#include "Waller.h"
#include "geometry_msgs/Twist.h"
#include <math.h>


Values::Values()
{

}


std::ostream& operator<<(std::ostream& os, const RobotState& state) {
    switch (state) {
        case RobotState::FORWARD: os << "forward"; break;
        case RobotState::TURN_LEFT: os << "turn_left"; break;
        case RobotState::FORWARD_LEFT: os << "forward_left"; break;
        case RobotState::FIND_WALL: os << "find_wall"; break;
    }
    return os;
}

Sensor::Sensor(float angle, float spread)
{
    left = ((angle - spread)* M_PI) / 180;
    right = ((angle + spread)* M_PI) / 180;
}

Sensor::Sensor() : left(0), right(0) {} 

Values Sensor::isFree(const sensor_msgs::LaserScan::ConstPtr& scan) const {
    bool isObstacleInFront = false;
        int minIndex = static_cast<int>(std::ceil((left - scan->angle_min) / scan->angle_increment));
        int maxIndex = static_cast<int>(std::floor((right - scan->angle_min) / scan->angle_increment));
        float average = 0.0f;

        for (int currIndex = minIndex + 1; currIndex <= maxIndex; ++currIndex) {
            float currentRange = scan->ranges[currIndex];

            // Проверяем, является ли значение NaN
            if (std::isnan(currentRange)) {
                average += scan->range_max;
            } else {
                average += currentRange;
            }

            if (currentRange < MIN_DIST_FROM_OBSTACLE) {
                isObstacleInFront = true;
                break;
            }
        }
        // Возвращаем пару: наличие препятствия и среднее значение
        float rangeCount = static_cast<float>(maxIndex - minIndex);
        Values values = Values();
        values.avg = float(average / rangeCount);
        values.isObstacleInFront = isObstacleInFront;
        return  values;
}

RobotData::RobotData()
{
    state = RobotState::FIND_WALL;
    right = Sensor(270,15);
    front = Sensor(180,15);

}

Stopper::Stopper()
{
    data = RobotData();
    keepMoving = true;
    right = 0;
    front = 0;
    rightAvg = 0;
    frontAvg = 0;

    // Advertise a new publisher for the robot's velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscribe to the simulated robot's laser scan topic
    laserSub = node.subscribe("scan", 1, &Stopper::scanCallback, this);
}

void RobotData::nextState() {
    if (state == RobotState::last) {
        state = RobotState::first;
    } else {
        state = static_cast<RobotState>(static_cast<int>(state) + 1);
    }
    std::cout << "STATE: " << state << std::endl;
    
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

float RobotData::getTurnAngle() {
    return (-90 * (M_PI / 180));
}

// Send a velocity command
void Stopper::moveForward() {
    geometry_msgs::Twist msg; 
    msg.angular.z = 0;
    // The default constructor will set all commands to 0
    msg.linear.x = (-1)*FORWARD_SPEED;
    commandPub.publish(msg);
}



void Stopper::rotate() {
    ros::Rate rate(10);
    geometry_msgs::Twist msg;
    ROS_INFO("Rotating");
    float turnAngle = data.getTurnAngle();

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

    data.state = RobotState::FORWARD;
    commandPub.publish(msg);
    ros::spinOnce();
    rate.sleep();
}

// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    bool isObstacleInFront = false;

    Values rightObj = data.right.isFree(scan);
    Values frontObj = data.front.isFree(scan);
    // std::cout << frontObj.avg << std::endl;

    if (data.state == RobotState::FIND_WALL) {
        if (frontObj.isObstacleInFront) {
            data.state = RobotState::TURN_LEFT;
        }
    }
    right = rightObj.isObstacleInFront;
    front = frontObj.isObstacleInFront;
    frontAvg = frontObj.avg;
    rightAvg = rightObj.avg; 

}

void Stopper::startMoving()
{
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    ros::Rate rate(10);
    ROS_INFO("StartSSSSSSSSSSSSSSSSSSSSSSS");

    // Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
    while (ros::ok()) {
        // if (data)
        if (data.state == RobotState::FIND_WALL) {
            moveForward();
        } else if (data.state == RobotState::TURN_LEFT) {
            rotate();
        } else if (data.state == RobotState::FORWARD){
            std::cout << frontAvg << std::endl;
            // std::cout << "START: " << end << std::endl;
            if (frontAvg > 0.4) {
                msg.angular.z = 0;
                msg.linear.x = (-1)*FORWARD_SPEED;
                commandPub.publish(msg);
                ros::spinOnce();
                rate.sleep();
            } else {
                ROS_INFO("LESS 0.4");
                msg.linear.x = 0;
                commandPub.publish(msg);
                rotate();
                ros::spinOnce();
                rate.sleep();
            }
            if (rightAvg > 0.42) {
                ROS_INFO("MORE 0.4");

                if (rightAvg > 0.6) {
                    msg.angular.z = (4)*ROTATE_SPEED / 3;
                    msg.linear.x = 0;
                    commandPub.publish(msg);
                }
                msg.angular.z = 1*ROTATE_SPEED / 3;
                msg.linear.x = 0;
                commandPub.publish(msg);
            } else if (rightAvg < 0.38) {
                msg.angular.z = (-1)*ROTATE_SPEED / 3;
                msg.linear.x = 0;
                commandPub.publish(msg);
            }
        }
            ros::spinOnce();
            rate.sleep();
    }
}
