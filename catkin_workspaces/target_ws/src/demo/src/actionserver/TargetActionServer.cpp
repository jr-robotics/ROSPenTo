#include <iostream>
#include <unistd.h>
#include "TargetActionServer.h"
using namespace std;

TargetActionServer::TargetActionServer()
{
  _feedback.actual.positions.resize(6);
  _feedback.desired.positions.resize(6);
  ros::NodeHandle n;
  _targetActionServer = new TTargetActionServer(n, "followJointTrajectory", boost::bind(&TargetActionServer::executeJointCommand, this, _1), false);
  cout << "[TargetActionServer] ... Target Action Server created." << endl;

  _targetActionServer->start();
  cout << "[TargetActionServer] ... Target Action Server started." << endl;
}

void TargetActionServer::executeJointCommand (const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  cout << "[TargetActionServer] ... Received Follow Joint Trajectory Action Goal: " << endl;
  
  for (const trajectory_msgs::JointTrajectoryPoint &point : goal->trajectory.points)
  {
    if (_targetActionServer->isPreemptRequested())
    {
      cout << "[TargetActionServer] ... action aborted on clients behalf" << endl;
      _targetActionServer->setPreempted();
      return;
    }

    cout << "[TargetActionServer] ... Heading towards [" << point.positions[0] << ", " << point.positions[1] << ", " << point.positions[2] << ", " << point.positions[3] << ", " << point.positions[4] << ", " << point.positions[5] << "]" << endl;
    sleep(1);
  }
  // _followJointTrajectoryActionServer->publishFeedback(feedback);

  cout << "[TargetActionServer] ... Action Goal succeeded." << endl;
  _result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  _targetActionServer->setSucceeded(_result);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TargetActionServer");
  TargetActionServer actionServer;
  ros::spin();

  return 0;
}
