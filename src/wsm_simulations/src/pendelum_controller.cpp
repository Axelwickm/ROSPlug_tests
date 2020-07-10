#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include <wsm_simulations/PendelumState.h>

#include <cmath>
#include <cstdio>

class Controller {
public:
    Controller(ros::NodeHandle n){
        this->n = n;
        pendelum_pub = n.advertise<std_msgs::Float32>("pendelum_motor", 2);
        pendelum_state = n.subscribe("pendelum_state", 1, &Controller::callback, this);
        ROS_INFO("Pendelum controller ready");
    }
    ~Controller(){
        pendelum_pub.shutdown();
        pendelum_state.shutdown();
    }

private:
    ros::NodeHandle n;
    ros::Publisher pendelum_pub;
    ros::Subscriber pendelum_state;
    unsigned count = 0;

    void callback(const boost::shared_ptr<const wsm_simulations::PendelumState> &msg){
        const float position = msg->position;
        const float angle = msg->angle;
        const float motor_response = motor_value(position, angle);
        std_msgs::Float32 response;
        response.data = motor_response;

        pendelum_pub.publish(response);

        ROS_INFO("position: %f, angle: %f -> motor response: %f (count %u)", position, angle, motor_response, count);

        count++;
    }

    float motor_value(const float position, const float angle) const {
        if (count%80 > 60)
            return 0;
        return sinf(count/10.f)*10.f;
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pendelum_controller");
  try {
      ros::NodeHandle n;
      Controller controller(n);

      ros::Rate rate(10.f);
      while (ros::ok()){
          ros::spinOnce();
          rate.sleep();
      }
  } catch (const std::exception& e) {
      printf("Error: %s\n", e.what());
      fflush(stdout);
  }

  return 0;
}