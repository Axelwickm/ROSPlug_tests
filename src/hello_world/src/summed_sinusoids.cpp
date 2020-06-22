#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <sstream>
#include <math.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "summed_sinusoids");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("summed_sinusoids", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    float t = count / 10.f;
    float y = sinf(t) + sinf(3*t)/3 + sinf(5*t)/5 + sinf(7*t)/7 + sinf(9*t)/9;
    std_msgs::Float32 msg;
    msg.data = y;

    ROS_INFO("%d - %f", count, msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }

  return 0;
}