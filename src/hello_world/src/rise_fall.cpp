#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rise_fall");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("rise_fall", 1000);

  ros::Rate loop_rate(10);

  const int max_value = 30;
  int count = 0;
  while (ros::ok())
  {
    std_msgs::Int32 msg;

    int i = count % (2*max_value);
    if (max_value < i)
        i = 2*max_value-i;
    msg.data = i;

    ROS_INFO("%d - %d", count, msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  chatter_pub.shutdown();
  return 0;
}