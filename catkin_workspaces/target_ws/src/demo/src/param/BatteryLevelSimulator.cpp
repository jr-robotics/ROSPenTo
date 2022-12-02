#include "ros/ros.h"
#include <iostream>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "BatteryLevelSimulator");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  int batteryLevel = 100;

  while (ros::ok() && (batteryLevel >= 0))
  {
    n.setParam("/battery_level", batteryLevel);
    cout << "[BatteryLevelSimulator] ... current battery level: " << batteryLevel << endl;
    // ros::spinOnce();
    loop_rate.sleep();
    batteryLevel -= 1;
  }

  return 0;
}


