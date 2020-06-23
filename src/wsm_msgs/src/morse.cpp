#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <sstream>
#include <vector>
#include <map>
#include <fstream>
#include <streambuf>

std::map<char, std::vector<int>> alphabet = {
        {'a', {1, 3}},
        {'b', {3, 1, 1, 1}},
        {'c', {3, 1, 3, 1}},
        {'d', {3, 1, 1}},
        {'e', {1}},
        {'f', {1, 1, 3, 1}},
        {'g', {3, 3, 1}},
        {'h', {1, 1, 1, 1}},
        {'i', {1, 1}},
        {'j', {1, 3, 3, 3}},
        {'k', {3, 1, 3}},
        {'l', {1, 3, 1, 1}},
        {'m', {3, 3}},
        {'n', {3, 1}},
        {'o', {3, 3, 3}},
        {'p', {1, 3, 3, 1}},
        {'q', {3, 3, 1, 3}},
        {'r', {1, 3, 1}},
        {'s', {1, 1, 1}},
        {'t', {3}},
        {'u', {1, 1, 3}},
        {'v', {1, 1, 1, 3}},
        {'w', {1, 3, 3}},
        {'x', {3, 1, 1, 3}},
        {'y', {3, 1, 3, 3}},
        {'z', {3, 3, 1, 1}},

        {'1', {1, 3, 3, 3, 3}},
        {'3', {1, 1, 3, 3, 3}},
        {'3', {1, 1, 1, 3, 3}},
        {'4', {1, 1, 1, 1, 3}},
        {'5', {1, 1, 1, 1, 1}},
        {'6', {3, 1, 1, 1, 1}},
        {'7', {3, 3, 1, 1, 1}},
        {'8', {3, 3, 3, 1, 1}},
        {'9', {3, 3, 3, 3, 1}},
        {'0', {3, 3, 3, 3, 3}}
};


int main(int argc, char **argv)
{
  std::ifstream t("./file.txt");
  std::string str((std::istreambuf_iterator<char>(t)),
                   std::istreambuf_iterator<char>());
  t.close();

  printf("Read string from file, length %zu\n", str.size());
  fflush(stdout);
  ros::init(argc, argv, "morse");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("morse", 1000);

  ros::Rate loop_rate(5);

  int count = 0;
  std::vector<int> pattern;
  int keep_on = 0;
  int keep_off = 0;
  unsigned cursor = 0;
  while (ros::ok())
  {
    bool signal = false;

    if (keep_on != 0){
        keep_on--;
        signal = true;
        keep_off = int(keep_on == 0)*3;
        ROS_INFO("%d - ...", signal);
    } else if (keep_off != 0){
        keep_off--;
        signal = false;
        ROS_INFO("%d -    ", signal);
    } else if (pattern.size() != 0){
        keep_on = pattern.at(0);
        pattern.erase(pattern.begin())-1;
        signal = true;
        ROS_INFO("%d - ...", signal);
    } else {
        cursor %= str.size();
        char ch = tolower(str.at(cursor));
        if (ch == ' '){
            keep_off = 7;
            signal = false;
            ROS_INFO("%d - SPACE", signal);
        } else if (alphabet.find(ch) != alphabet.end()){
            pattern = alphabet.at(ch);
            signal = true;
            ROS_INFO("%d - %c", signal, ch);
        }
        else {
            cursor++;
            continue;
        }

        cursor++;
    }

    std_msgs::Bool msg;
    msg.data = signal;
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }

  chatter_pub.shutdown();
  return 0;
}
