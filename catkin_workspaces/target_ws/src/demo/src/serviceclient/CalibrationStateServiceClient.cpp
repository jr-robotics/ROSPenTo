#include "ros/ros.h"
#include "control_msgs/QueryCalibrationState.h"
#include <iostream>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CalibrationStateServiceClient");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  ros::ServiceClient calibrationStateClient = n.serviceClient<control_msgs::QueryCalibrationState>("calibration_state");
  control_msgs::QueryCalibrationState srv;

  while (ros::ok())
  {
    if (calibrationStateClient.call(srv))
    {
      if (srv.response.is_calibrated)
      {
        cout << "[CalibrationStateServiceClient] ... robot is calibrated" << endl;
      }
      else
      {
        cout << "[CalibrationStateServiceClient] ... robot is not calibrated" << endl;
      }
    }
    else
    {
      cout << "[CalibrationStateServiceClient] ... error while calling service server" << endl;
    }
    
    // ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
