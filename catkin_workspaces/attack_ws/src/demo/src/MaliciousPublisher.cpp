#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MaliciousPublisher");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;

  cout << "[MaliciousPublisher] ... started publishing" << endl;

  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Message from malicious publisher " << count;
    msg.data = ss.str();
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}


