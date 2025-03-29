#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <simple_counter/CounterAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_counter");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<simple_counter::CounterAction> ac("counter", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  simple_counter::CounterGoal goal;
  if (argc < 3) {
    ROS_ERROR("Not enough arguments");
    return 1;
  }
  int start = std::stoi(argv[1]);
  int end = std::stoi(argv[2]);
  ROS_INFO("Received arguments: %d and %d", start, end);

  goal.start = start;
  goal.end = end;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}