#ifndef TARGET_ACTION_SERVER_H
#define TARGET_ACTION_SERVER_H

#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/simple_action_server.h"

class TargetActionServer
{
  public:
    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> TTargetActionServer;
    typedef control_msgs::FollowJointTrajectoryFeedback TFeedback;
    typedef control_msgs::FollowJointTrajectoryResult TResult;

    TargetActionServer();
    void executeJointCommand (const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
  
  private:
    TTargetActionServer *_targetActionServer;
    TFeedback _feedback;
    TResult _result;

    bool isPreemptRequested();
    void setPreempted();
};

#endif // TARGET_ACTION_SERVER_H
