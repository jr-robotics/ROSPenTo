#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
using namespace std;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  cout << "[TargetSubscriber] ... " << msg->data.c_str() << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TargetSubscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}

