#include "../include/wsm_simulations/WSM_Qtable.h"
#include "QTable.hpp"

#include <cmath>

extern "C" {
    EXPORT void* QTableConstructor(double learning_rate, double epsilon, double discount_factor){
        //QTable::ActionsSettings action_setting = {wsm_action_setting.count, wsm_action_setting.minimum, wsm_action_setting.maximum}; FIXME

        QTable::StateSettings state_settings = {
                {9,  -2, 2},           // Velocity
                {15, -15, 15},        // AngularVelocity
                {15, -M_PI, M_PI},   // Angle
            };
        QTable::ActionsSettings action_setting = {3, -14, 14};
        QTable* qtable = new QTable(state_settings, action_setting, learning_rate, epsilon, discount_factor);
        return static_cast<void*>(qtable);
    }

    EXPORT void QTableDestructor(void* object){
        delete static_cast<QTable*>(object);
    }

    EXPORT double choose_action(void* object, double* raw_state, size_t state_dimensions, double reward,
                double learning_rate, double epsilon, double discount_factor){
        QTable* qtable = static_cast<QTable*>(object);
        qtable->learning_rate = learning_rate;
        qtable->epsilon = epsilon;
        qtable->discount_factor = discount_factor;

        std::vector<float> raw_state_vec(state_dimensions);
        for (size_t i = 0; i < state_dimensions; i++){
            raw_state_vec.at(i) = *(raw_state+i);
        }

        const QTable::State state = qtable->get_state(raw_state_vec);
        std::tuple<float, QTable::ActionSpace::iterator> action = qtable->choose_action(state);
        qtable->update_action(static_cast<float>(reward), state);

        return std::get<0>(action);
    }

    EXPORT void reset(void* object){
        QTable* qtable = static_cast<QTable*>(object);
        qtable->reset();
    }
}
