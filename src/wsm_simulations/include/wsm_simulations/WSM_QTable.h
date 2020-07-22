#ifndef WSN_QTABLE_H
#define WSN_QTABLE_H


#if defined(_MSC_VER)
//  Microsoft VC++
    #define EXPORT __declspec(dllexport)
#else
//  GCC
#define EXPORT __attribute__((visibility("default")))
#endif


/*struct WSM_StateSettings {
    double minimum;
    double maximum;
    int count;
};*/


extern "C" {
    //EXPORT void* QTableConstructor(WSM_StateSettings* wsm_action_setting); // FIXME
    EXPORT void* QTableConstructor(double learning_rate, double epsilon, double discount_factor);
    EXPORT void QTableDestructor(void* object);

    EXPORT double choose_action(void* object, double* raw_state, size_t state_dimensions, double reward,
                double learning_rate, double epsilon, double discount_factor);

    // Will not update from last action
    EXPORT void reset(void* object);
}

#endif