#include "../include/wsm_simulations/WSM_Qtable.h"
#include "QTable.hpp"

#include <cmath>
#include <vector>
#include <map>

// This is needed in order to read and write betwen different DLLs
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    #include <windows.h>
    HANDLE ProcessHeapHandle = GetProcessHeap();
    #define ROSPLUG_ALLOC(X) HeapAlloc(ProcessHeapHandle, HEAP_GENERATE_EXCEPTIONS, (X))
    #define ROSPLUG_FREE(X) HeapFree(ProcessHeapHandle, 0, (X))
#else
    #define ROSPLUG_ALLOC(X) malloc((X))
    #define ROSPLUG_FREE(X) free((X))
#endif

typedef struct {
    void* a;
    std::size_t s;
} wsm_external_array_t;

std::map<void*, wsm_external_array_t*> qtable_external_arrays;

#include <chrono>
#include <thread>

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
        printf("Constructed qtable\n"); fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(750));
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

    EXPORT void allocate_EA(void* object){ // EA = External Array
        try {
            QTable* qtable = static_cast<QTable*>(object);
            std::size_t s = get_QTable_data_size(object);
            void* ea_memory = ROSPLUG_ALLOC(s);
            auto wsm_array = static_cast<wsm_external_array_t*>(ROSPLUG_ALLOC(sizeof(wsm_external_array_t)));
            wsm_array->a = ea_memory;
            wsm_array->s = s;
            printf("Allocated external array for qtable %p, allocated block %p, size %d\n",
                   wsm_array, wsm_array->a, wsm_array->s);
            qtable_external_arrays.insert(std::make_pair(object, wsm_array));
        } catch (const std::exception e){
            fprintf(stderr, "Allocate external array failed: %s", e.what());
            fflush(stderr);
            throw;
        }
    }

    EXPORT int get_QTable_data_size(void* object){
        QTable* qtable = static_cast<QTable*>(object);
        return (1+qtable->explored.size())*(4+qtable->state_settings.size())*sizeof(int32_t);
    }

    EXPORT int get_low_int_EA(void* object){
        wsm_external_array_t *wsm_array = qtable_external_arrays.at(object);
        uint64_t int_ptr = reinterpret_cast<uint64_t>(wsm_array);
        int low;
        std::memcpy(&low, &int_ptr, sizeof(low));
        return low;
    }

    EXPORT int get_high_int_EA(void* object){
        wsm_external_array_t *wsm_array = qtable_external_arrays.at(object);
        uint64_t int_ptr = reinterpret_cast<uint64_t>(wsm_array) >> 32;
        int high;
        std::memcpy(&high, &int_ptr, sizeof(high));
        return high;
    }

    EXPORT void set_array_data(void* object, int table_id){
        // Build representation of qtable sutiable for WSM and ROS message
        QTable* qtable = static_cast<QTable*>(object);
        const std::size_t state_count = qtable->state_settings.size();
        wsm_external_array_t *wsm_array = qtable_external_arrays.at(object);
        const std::size_t required_size = get_QTable_data_size(object);

        if (wsm_array->s != required_size){
            ROSPLUG_FREE(wsm_array->a);
            wsm_array->a = ROSPLUG_ALLOC(required_size);
            wsm_array->s = required_size;
        }
        int32_t *data = static_cast<int32_t*>(wsm_array->a);

        // Write table size first in data
        *(data+0) = qtable->explored.size();
        *(data+1) = qtable->action_count;
        *(data+2) = qtable->state_settings.size();
        int32_t *state_addr = data+4;
        for (auto &stateSetting : qtable->state_settings){
            *state_addr = std::get<0>(stateSetting);
            state_addr++;
        }

        // Write table values
        std::size_t i = 1;
        for (auto &e : qtable->explored){
            // Extract values
            QTable::State state = e.first;
            unsigned action = state.back();
            state.pop_back();
            double qvalue = static_cast<double>(qtable->qtable.at(state).at(action));
            unsigned explored_count = e.second;

            // Write values
            memcpy(data+i*(state_count+4)+0, &qvalue, sizeof(qvalue)); // Takes up 2 int32_t
            memcpy(data+i*(state_count+4)+2, &explored_count, sizeof(explored_count));
            memcpy(data+i*(state_count+4)+3, &action, sizeof(action));
            std::copy(state.begin(), state.end(), data+i*(state_count+4)+4);
            i++;
        }
    };

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
