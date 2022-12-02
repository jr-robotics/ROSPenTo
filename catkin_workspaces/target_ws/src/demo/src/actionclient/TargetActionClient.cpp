#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
using namespace std;

double get_random_double()
{
  return ceil((((double)rand() / RAND_MAX) * M_PI) * 100.0) / 100.0;
}

trajectory_msgs::JointTrajectoryPoint generateTrajectoryPoint()
{
  trajectory_msgs::JointTrajectoryPoint point;

  point.positions = {get_random_double(), get_random_double(), get_random_double(), get_random_double(), get_random_double(), get_random_double()};
  point.velocities = {get_random_double(), get_random_double(), get_random_double(), get_random_double(), get_random_double(), get_random_double()};
  point.accelerations = {get_random_double(), get_random_double(), get_random_double(), get_random_double(), get_random_double(), get_random_double()};

  return point;
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    cout << "[TargetActionClient] ... please provide number of points" << endl;
    return 0;
  }

  int number_of_points = atoi(argv[1]);

  ros::init(argc, argv, "TargetActionClient");
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> actionClient("followJointTrajectory", true);
  actionClient.waitForServer();
  cout << "[TargetActionClient] ... connected to action client" << endl;
  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = {"Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"};
  vector<trajectory_msgs::JointTrajectoryPoint> pointVec;
  trajectory_msgs::JointTrajectoryPoint p1 = generateTrajectoryPoint();
  trajectory_msgs::JointTrajectoryPoint p2 = generateTrajectoryPoint();
  string key;

  for (int i=0; i<number_of_points; ++i)
  {
    pointVec.push_back(generateTrajectoryPoint());
  }

  trajectory.points = pointVec;
  goal.trajectory = trajectory;
  actionClient.sendGoal(goal);

  cout << "[TargetActionClient] ... do you want to cancel the action goal regularly [y/n]?" << endl;
  cin >> key;

  if (key == "y")
  {
      cout << "[TargetActionClient] ... canceling goal" << endl;
      actionClient.cancelGoal();
  }
  else if (key == "n")
  {
      cout << "[TargetActionClient] ... goal will not be canceled" << endl;
  }
  else
  {
      cout << "[TargetActionClient] ... wrong key, ignoring..." << endl;
  }

  bool finished_before_timeout = actionClient.waitForResult(ros::Duration(number_of_points + 5));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = actionClient.getState();
    cout << "[TargetActionClient] ... Action finished: " << state.toString().c_str() << endl;
  }
  else
  {
    actionlib::SimpleClientGoalState state = actionClient.getState();
    cout << "[TargetActionClient] ... Action did not finish before timeout: " << state.toString().c_str() << endl;

  }

  return 0;
}
