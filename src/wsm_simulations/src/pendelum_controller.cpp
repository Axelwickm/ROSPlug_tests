#include "QTable.hpp"

#include "ros/ros.h"
#include "wsm_simulations/PendelumMotor.h"
#include <wsm_simulations/PendelumState.h>
#include <cstdio>
#include <memory>
#include <fstream>

class Controller {
public:
    Controller(ros::NodeHandle n, const QTable &qtable,
            bool should_save, const std::string filename, unsigned save_interval)
            : qtable(qtable), n(n), should_save(should_save), filename(filename), save_interval(save_interval){
        pendelum_pub = n.advertise<wsm_simulations::PendelumMotor>("pendelum_motor", 2);
        pendelum_state = n.subscribe("pendelum_state", 1, &Controller::callback, this);

        ROS_INFO("Pendelum controller ready");
    }

    ~Controller(){
        pendelum_pub.shutdown();
        pendelum_state.shutdown();
    }

    QTable qtable;
private:
    ros::NodeHandle n;
    ros::Publisher pendelum_pub;
    ros::Subscriber pendelum_state;

    bool should_save;
    const std::string filename;
    unsigned save_interval;

    unsigned count = 0;
    unsigned last_explored_states = 0;
    float new_states_rate = 0;
    ros::Time last_time;
    float last_position = 0;
    float last_angle = 0;
    float average_reward = 0;

    QTable::ActionSpace::iterator last_action;


    void callback(const boost::shared_ptr<const wsm_simulations::PendelumState> &msg){
        const float position = msg->position;
        const float angle = remainderf(msg->angle, 2.0f*M_PI);
        double delta_time = 0;
        if (count != 0){
            delta_time = msg->delta_time;
        }

        float velocity = (position - last_position) / (delta_time == 0 ? 1 : delta_time);
        float delta_angle = remainderf(last_angle-angle, 2.0f*M_PI);
        float angular_velocity = delta_angle / (delta_time == 0 ? 1 : delta_time);

        const std::vector<float> raw_state{velocity, angular_velocity, angle};
        const QTable::State &state = qtable.get_state(raw_state);
        float qvalue = 0;
        float r = 0;
        if (count != 0){
            r = reward(angle, velocity, angular_velocity);
            qvalue = qtable.update_action(last_action, r, state);
            //printf("Got reward %f, qvalue is %f\n", r, qvalue);
        }

        if (should_save && count%save_interval == 0){
              std::ofstream file;
              file.open(filename, std::ios::binary | std::ios::out);
              qtable.write(file);
              file.close();
              qtable.print();
        }
        new_states_rate = new_states_rate*0.98f + float(qtable.explored.size()-last_explored_states)*0.02f;
        average_reward = average_reward*0.995+r*0.005;
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
        response.average_reward = average_reward;
        pendelum_pub.publish(response);

        qtable.epsilon = (sinf(count/500.0f)*0.5f+0.5f)*(8000.0f/float(3000.0f+count*0.4f))*0.08f;

        printf("State {%d %d %d} - qvalue %f\n", state[0], state[1], state[2], qvalue);
        printf("Epsilon %f\n", qtable.epsilon);
        printf("New states rate %f\n", new_states_rate);
        ROS_INFO("(delta_time %f, position: %f), velocity %f, angle: %f, angular_velocity %f -> motor response: %f (count %u)",
                delta_time, position, velocity, angle, angular_velocity, motor_response, count);
        printf("\n");


        count++;
        last_time = ros::Time::now();
        last_explored_states = qtable.explored.size();
        last_position = position;
        last_angle = angle;
        last_action = std::get<1>(action);
    }

    inline float reward(float angle, float velocity, float angular_velocity){
        //angle = remainderf(angle + 3.0f*M_PI, 2.0f*M_PI);
        float r = -powf(fabs(angle)*0.5, 1.2f) - 0.3f*logf(fabs(angular_velocity)+1.0) - 0.05*fabs(velocity);
        //return fabs(angular_velocity);
        return fmaxf(fminf(r, 100), -100.f);
    }
};


int main(int argc, char **argv)
{
    std::string filename = "pendelum_controller.qtable";
    bool load_from_file = false;
    bool save_to_file = false;
    unsigned saving_interval = 1000;

    // Prase input arguments
    for (int i = 1; i < argc; i++){
        std::string arg(argv[i]);
        if (arg == "-h" || arg == "--help"){
            printf("--- Inverse pendelum contoller which subscribes to topic pendelum_state\n"
                   "--- and outputs in topic pendelum_motor. Controlled through q-learning.\n"
                   "--- Options:\n"
                   "--- \t-h, --help     : show this help\n"
                   "--- \t-l, --load     : load model from file (default: false)\n"
                   "--- \t-s, --save     : save model to file at intervals (default: false)\n"
                   "--- \t-i, --inter #  : save model to file at intervals (default: false)\n"
                   "--- \t-f, --file \"\"  : file to load and save to (default: \"%s\")\n"
                   , filename.c_str());
            return 0;
        }
        else if (arg == "-l" || arg == "--load"){
            load_from_file = true;
        }
        else if (arg == "-s" || arg == "--save"){
            save_to_file = true;
        }
        else if (arg == "-i" || arg == "--inter"){
            saving_interval = std::atoi(argv[i+1]);
            i++;
        }
        else if (arg == "-f" || arg == "--file"){
            filename.assign(argv[i+1]);
            i++;
        }
    }

    // Start ROS
    ros::init(argc, argv, "pendelum_controller");
    try {
        ros::NodeHandle n;
        std::unique_ptr<Controller> controller;

        if (load_from_file){
            std::ifstream file(filename, std::ios::in | std::ios::binary);
            if (!file.is_open()){
                throw std::runtime_error("File could not be opened. Does it exist?");
            }
            controller.reset(new Controller(n, QTable::read(file), save_to_file, filename, saving_interval));
        }
        else {
            QTable::StateSettings state_settings = {
                {9,  -2, 2},           // Velocity
                {15, -15, 15},        // AngularVelocity
                {15, -M_PI, M_PI},   // Angle
            };
            QTable::ActionsSettings actions = {3, -10, 10};
            QTable qtable(state_settings, actions, 0.05, 0.001, 0.99);
            controller.reset(new Controller(n, qtable, save_to_file, filename, saving_interval));
        }

        controller->qtable.print();

        // Start waiting for topic
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