#include "ros/ros.h"

#include <wsm_msgs/StringArray.h>

#include <sstream>
#include <vector>
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "strs_node");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<wsm_msgs::StringArray>("strs", 1000);

  ros::Rate loop_rate(0.5);

  const std::vector<std::string> str_list =
          {"Lorem", " ipsum ", "dolor ", "sit ", "amet, ", "ius", "an ", "animalm ", "euismod."};
  std::vector<std::string> temp_str_list;

  int count = 0;
  while (ros::ok())
  {
    if (str_list.size() != temp_str_list.size()){
        std::string str = str_list.at(count%str_list.size());
        temp_str_list.push_back(str);
    } else {
        temp_str_list.clear();
    }

    wsm_msgs::StringArray msg;
    msg.data = temp_str_list;

    ROS_INFO("%d - %d", count, msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }

  chatter_pub.shutdown();
  return 0;
}