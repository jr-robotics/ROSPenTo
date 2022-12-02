#include "ros/ros.h"
#include "control_msgs/QueryCalibrationState.h"
#include <stdlib.h>
#include <time.h>
#include <iostream>
using namespace std;


bool calibrationStateCallback(control_msgs::QueryCalibrationState::Request &req, control_msgs::QueryCalibrationState::Response &res)
{
  int random = rand() % 2;
  
  if (random == 0)
  {
    res.is_calibrated = false;
  }

  if (random == 1)
  {
    res.is_calibrated = true;
  }
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CalibrationStateServiceServer");
  ros::NodeHandle n;
  srand(time(NULL));
  ros::ServiceServer calibrationStateServer = n.advertiseService("calibration_state", calibrationStateCallback);
  cout << "[CalibrationStateServiceServer] ... started service" << endl;
  ros::spin();

  return 0;
}
