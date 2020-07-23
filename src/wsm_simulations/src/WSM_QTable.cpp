#include "../include/wsm_simulations/WSM_Qtable.h"
#include "QTable.hpp"

#include <cmath>

extern "C" {
    EXPORT void* QTableConstructor(const char* filepath_chars){
        const std::string filepath(filepath_chars);
        //QTable::ActionsSettings action_setting = {wsm_action_setting.count, wsm_action_setting.minimum, wsm_action_setting.maximum}; FIXME


        QTable::StateSettings state_settings = {
                {9,  -2, 2},         // Velocity
                {17, -15, 15},       // AngularVelocity
                {23, -M_PI, M_PI},   // Angle
            };
        QTable::ActionsSettings action_setting = {7, -18, 18};
        QTable* qtable;
        if (filepath.empty()){
            printf("Create new table\n"); fflush(stdout);
            qtable = new QTable(state_settings, action_setting, 0, 0, 0);
        } else {
            printf("Load qtable from %s\n", filepath_chars); fflush(stdout);
            std::ifstream file(filepath, std::ios::in | std::ios::binary);
            if (!file.is_open()){
                fprintf(stderr, "File could not be opened. Does it exist?\n"); fflush(stderr);
                throw std::runtime_error("File could not be opened. Does it exist?");
            }
            qtable = new QTable(QTable::read(file));
            qtable->print();
        }
        return static_cast<void*>(qtable);
    }

    EXPORT void QTableDestructor(void* object){
        delete static_cast<QTable*>(object);
    }

    EXPORT void save(void* object, const char* filepath_chars){
        QTable* qtable = static_cast<QTable*>(object);
        std::ofstream file;
        file.open(std::string(filepath_chars), std::ios::binary | std::ios::out);
        qtable->write(file);
        file.close();

        printf("Saved qtable to: \"%s\"\n", filepath_chars);
        qtable->print();
        fflush(stdout);
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
        qtable->update_action(static_cast<float>(reward), state);
        std::tuple<float, QTable::ActionSpace::iterator> action = qtable->choose_action(state);
        return std::get<0>(action); // Return action
    }

    EXPORT double get_qvalue(void* object){
        QTable* qtable = static_cast<QTable*>(object);
        return static_cast<double>(*qtable->last_action);
    }

    EXPORT void reset(void* object){
        QTable* qtable = static_cast<QTable*>(object);
        qtable->reset();
    }
}
