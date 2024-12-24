#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include <iostream>

using namespace std;

static int a = 0;
static double poseStart;
static double poseCurrent;

void poseCallback(const turtlesim::PoseConstPtr& msg) // Topic messages callback
{
  if (a==0) {
  	poseStart = msg->x;
  	a=1;
  }
  poseCurrent = msg->x;
  ROS_INFO("x: %.2f, y: %.2f", msg->x, msg->y);
}


int main(int argc, char ** argv)
{

    bool success = true;
    const double FORWARD_SPEED_MPS = 2.0;
    int16_t DISTANCE = 6;
    int16_t HORIZONTAL_DISTANCE = 3;
    
    string robot_name = string(argv[1]);

    ros::init(argc, argv, "move_turtle");   //Initialize the node
    ros::NodeHandle node;

    // сервис для мгновенной телепортации черепашки
    ros::service::waitForService("turtle1/teleport_absolute");
    ros::ServiceClient teleportClient = node.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
    
    // сервис для отключения рисование пути и изменения его цвета
    ros::service::waitForService("turtle1/set_pen");
    ros::ServiceClient setpenClient = node.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
    
    // сервис для очистки холста
    //ros::service::waitForService("clear");
    // ros::ServiceClient clearClient = node.serviceClient<std_srvs::Empty>("clear");


    // контейнер аргументов для вызова сервисов
    turtlesim::TeleportAbsolute teleport_arguments; 
    turtlesim::SetPen pen_arguments;

    // выключаем рисованиу пути, чтобы при телепортации черепашки не оставить след
    pen_arguments.request.off = 1;

    // выполняем запрос отключения рисования
    success = setpenClient.call(pen_arguments);

    if (!success) {
      ROS_ERROR_STREAM("Pen failed to switch off");
    }

    teleport_arguments.request.x = 3.0; // позиция по х
    teleport_arguments.request.y = 4.0; // позиция по у
    teleport_arguments.request.theta = 0; // 90 градусов

    // выполняем телепортацию черепашки
    success = teleportClient.call(teleport_arguments);
    
    if (!success) {
    ROS_ERROR_STREAM("Turtle failed to teleport" );
    }

    // Включаем рисование пути обратно и меняем цвет рисовки 
    pen_arguments.request.off = 0;
    pen_arguments.request.r = 240;
    pen_arguments.request.g = 45;
    pen_arguments.request.b = 240;
    pen_arguments.request.width = 16;
    success = setpenClient.call(pen_arguments);
    if (!success) {
    ROS_ERROR_STREAM("TurtlePen failed to switch on");
    }


    // A publisher for the movement data
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 10);

    // A listener for pose
    ros::Subscriber sub = node.subscribe(robot_name + "/pose", 10, poseCallback);

    // Drive forward at a given speed. The robot points up the x-axis.
    // The default constuctor will set all commands to 0.
    geometry_msgs::Twist msg;
    msg.linear.y = FORWARD_SPEED_MPS;
    msg.angular.z = 0.0;

    // Loop at 10Hz, publishing movement conmmands until we shut down.
    ros::Rate rate(10);
    ROS_INFO("Starting to move forward");
    
    while (ros::ok())
    {

      // Левая сторона Н
      int16_t curDistance = 0;
      double t0 = ros::Time::now().toSec();
      cout << curDistance << endl;
      while (curDistance < DISTANCE) {
        double t1 = ros::Time::now().toSec();
        curDistance = abs((t1-t0)*msg.linear.y);
        pub.publish(msg);
      }
      ROS_INFO("STOP");

      // Возвращение к центру линии
      msg.linear.y = -FORWARD_SPEED_MPS;
      curDistance = 0;
      t0 = ros::Time::now().toSec();
      while (curDistance < DISTANCE / 2) {
        double t1 = ros::Time::now().toSec();
        curDistance = abs((t1-t0)*msg.linear.y);
        pub.publish(msg);
      }
      ROS_INFO("STOP");

      // Рисовка второго элемента Н
      msg.linear.y = 0;
      msg.linear.x = FORWARD_SPEED_MPS;
      curDistance = 0;
      t0 = ros::Time::now().toSec();
      while (curDistance < HORIZONTAL_DISTANCE) {
        double t1 = ros::Time::now().toSec();
        curDistance = abs((t1-t0)*msg.linear.x);
        pub.publish(msg);
      }

      // Рисовка верхней половины правой линии Н
      msg.linear.x = 0;
      msg.linear.y = FORWARD_SPEED_MPS;
      curDistance = 0;
      t0 = ros::Time::now().toSec();
      while (curDistance < DISTANCE / 2) {
        double t1 = ros::Time::now().toSec();
        curDistance = abs((t1-t0)*msg.linear.y);
        pub.publish(msg);
      }
      ROS_INFO("STOP");


      // Риcовка оставшейся нижней части
      msg.linear.y = -FORWARD_SPEED_MPS;
      curDistance = 0;
      t0 = ros::Time::now().toSec();
      while (curDistance < DISTANCE) {
        double t1 = ros::Time::now().toSec();
        curDistance = abs((t1-t0)*msg.linear.y);
        pub.publish(msg);
      }
      ROS_INFO("STOP");

      msg.linear.y = 0;

      pub.publish(msg);
      ros::spinOnce(); // Allow processing of incoming messages
      rate.sleep();
    }

    return 0;
}
