#include "ros/ros.h"
#include "std_msgs/String.h"
#include "actionlib_msgs/GoalID.h"
#include <iostream>
using namespace std;

void cancelCallback(const actionlib_msgs::GoalID::ConstPtr& msg)
{
  cout << "[GoalCancel] ... " << msg->id.c_str() << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GoalCancel");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("followJointTrajectory/cancel", 1000, cancelCallback);
  ros::spin();

  return 0;
}

