#include "ros/ros.h"
#include <iostream>
using namespace std;

int batteryLevel;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "BatteryLevelMonitor");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    n.getParamCached("battery_level", batteryLevel);
    {
      if (batteryLevel <= 50)
      {
        cout << "[BatteryLevelMonitor] ... current battery level: " << batteryLevel << " [WARNING]" << endl;
      }
      else
      {
        cout << "[BatteryLevelMonitor] ... current battery level: " << batteryLevel << " [OK]" << endl;
      }
    }

    // ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


