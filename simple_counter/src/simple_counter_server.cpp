#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <simple_counter/CounterAction.h>

#include <iostream>
#include <vector>
using namespace std;

class CounterAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<simple_counter::CounterAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  simple_counter::CounterFeedback feedback_;
  simple_counter::CounterResult result_;

public:

  CounterAction(std::string name) :
    as_(nh_, name, boost::bind(&CounterAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~CounterAction(void)
  {
  }

  bool isPrime(int n) {
    if (n < 2) { 
      return false;
    }
    for (int i = 2; i * i <= n; i++) {
        if (n % i == 0) {
          return false;
        }
    }
    return true;
}

vector<int> getNthPrimes(int start, int end) {
    vector<int> primes;
    int count = 1;
    int num = 2;

    while (count <= end) {
        if (isPrime(num)) {
          // ROS_INFO("NEW PRIME: %i", num);
          if (count >= start) {
            primes.push_back(num);
          } 
          count += 1;
        }
        num++;
    }
    return primes;
}


  void executeCB(const simple_counter::CounterGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(0.5);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();

    // publish info to the console for the user
    ROS_INFO("%s: Executing, printing  %i nth and  %i nth prime numbers", action_name_.c_str(), goal->start, goal->end);

    // start executing the action

    vector<int> primes = getNthPrimes(goal->start, goal->end);
    // ROS_INFO("%s: ALL PRIMES: %i",primes );


    for(int i=0; i<primes.size(); i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(primes[i]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
  };



int main(int argc, char** argv)
{
  ros::init(argc, argv, "counter");

  CounterAction counter("counter");
  ros::spin();

  return 0;
}
