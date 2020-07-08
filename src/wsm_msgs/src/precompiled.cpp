#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <wsm_msgs/Cities.h>

#include <sstream>
#include <map>
#include <vector>

struct City {
    std::string name;
    std::int64_t population;
    bool by_sea;
};

const std::map<std::string, std::vector<City>> countries = {
        {"sweden", {{"Stockholm", 939000, true}, {"Gothenburg", 550000, true}, {"Falun", 37000, false}}},
        {"united states", {{"Washington D.C.", 706000, false}, {"New York", 8623000, true}, {"Seattle", 744500, true}}},
        {"kenya", {{"Nairobi", 4000000, false}, {"Marsabit", 15400, false}, {"Malindi", 207000, true}}},
        {"india", {{"New Delhi", 11300000, false}, {"Mumbai", 12440000, true}}},
        {"switzerland", {{"geneva", 199000, false}}},
        {"monaco", {}} // City state
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "precompiled");

  ros::NodeHandle n;

  //ros::Publisher chatter_pub = n.advertise<wsm_msgs::City>("city", 5);
  ros::Publisher chatter_pub = n.advertise<wsm_msgs::Cities>("cities", 5);

  ros::Rate loop_rate(0.8);

  int count = 0;
  auto it = countries.begin(); // Not in instantiated order
  while (ros::ok())
  {
    wsm_msgs::Cities msg;
    msg.header.stamp = ros::Time::now();
    msg.header.seq = count;
    msg.header.frame_id = "_";
    msg.country = it->first;
    for (auto &city : it->second){
        wsm_msgs::City cityMsg;
        cityMsg.name = city.name;
        cityMsg.population = city.population;
        cityMsg.by_sea = city.by_sea;
        msg.cities.push_back(cityMsg);
        if (false && count % 3 == 0){
            chatter_pub.publish(cityMsg);
            ROS_INFO("%d", count);
            break;
        }

    }

    ROS_INFO("%d - %s (%zu)", count, it->first.c_str(), msg.cities.size());


    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
    it++;
    if (it == countries.end()){
        it = countries.begin();
    }
  }

  chatter_pub.shutdown();
  return 0;
}