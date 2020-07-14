#include "QTable.hpp"

#include "ros/ros.h"
#include "wsm_simulations/PendelumMotor.h"
#include <wsm_simulations/PendelumState.h>
#include <cstdio>


class Controller {
public:
    Controller(ros::NodeHandle n, const QTable::StateSettings &state_settings, const QTable::ActionsSettings &actions,
            const float learning_rate, const float epsilon, const float discount_factor)
    : qtable(state_settings, actions, learning_rate, epsilon, discount_factor){
        this->n = n;
        pendelum_pub = n.advertise<wsm_simulations::PendelumMotor>("pendelum_motor", 2);
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
    ros::Time last_time;
    float last_position = 0;
    float last_angle = 0;

    QTable qtable;
    QTable::ActionSpace::iterator last_action;

    void callback(const boost::shared_ptr<const wsm_simulations::PendelumState> &msg){
        const float position = msg->position;
        const float angle = remainderf(msg->angle, 2.0f*M_PI);
        double delta_time = 0;
        if (count != 0){
            delta_time = msg->delta_time;
        }

        float velocity = (position - last_position) / (delta_time == 0 ? 1 : delta_time);
        float angular_velocity = (angle - last_angle) / (delta_time == 0 ? 1 : delta_time);

        const std::vector<float> raw_state{velocity, angular_velocity, angle};
        const QTable::State &state = qtable.get_state(raw_state);
        float qvalue = 0;
        float r = 0;
        if (count != 0){
            r = reward(angle, angular_velocity);
            qvalue = qtable.update_action(last_action, r, state);
            //printf("Got reward %f, qvalue is %f\n", r, qvalue);
        }

        std::tuple<float, QTable::ActionSpace::iterator> action = qtable.choose_action(state);

        //printf("action: %f\n", std::get<0>(action));
        printf("Explored %u / %I32u\n", qtable.explored.size(), qtable.max_states);
        const float motor_response = std::get<0>(action);
        wsm_simulations::PendelumMotor response;
        response.motor_response = motor_response;
        response.reward = r;
        response.qvalue = qvalue;
        response.state = {state[0], state[1], state[2]};
        response.explored_states = qtable.explored.size();
        pendelum_pub.publish(response);

        printf("State {%d %d %d} - qvalue %f\n", state[0], state[1], state[2], qvalue);

        ROS_INFO("(delta_time %f, position: %f), velocity %f, angle: %f, angular_velocity %f -> motor response: %f (count %u)",
                delta_time, position, velocity, angle, angular_velocity, motor_response, count);
        printf("\n");

        count++;
        last_time = ros::Time::now();
        last_position = position;
        last_angle = angle;
        last_action = std::get<1>(action);
    }

    inline float reward(float angle, float angular_velocity){
        float r = -(powf(fabs(angle)*20.f, .5f) + 0.0f*powf(fabs(angular_velocity), 0.2f));
        return fmaxf(fminf(r, 100), -100.f);
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pendelum_controller");
  try {
      ros::NodeHandle n;
      QTable::StateSettings state_settings = {
              {3, -2, 2},         // Velocity
              {15, -20, 20},      // AngularVelocity
              {15, -M_PI, M_PI},  // Angle
      };
      QTable::ActionsSettings actions = {4, -12, 12};
      Controller controller(n, state_settings, actions, 0.2, 0.1, 0.1);

      ros::Rate rate(100.f);
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