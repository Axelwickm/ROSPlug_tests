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
    EXPORT void* QTableConstructor(const char* filepath, double* state_ranges, int* state_counts, size_t state_dims,
                                   double* action_range, int action_count);
    EXPORT void QTableDestructor(void* object);
    EXPORT void save(void* object, const char* filepath_chars);
    EXPORT void allocate_EA(void* object);
    EXPORT void deallocate_EA(void* object); // Deallocates all associated EAs
    EXPORT int get_QTable_data_size(void* object);
    EXPORT int get_low_int_EA(void* object);
    EXPORT int get_high_int_EA(void* object);
    EXPORT void set_array_data(void* object, int table_id);

    EXPORT double choose_action(void* object, double* raw_state, size_t state_dimensions, double reward,
                double learning_rate, double epsilon, double discount_factor);
    EXPORT double get_qvalue(void* object);

    // Will not update from last action
    EXPORT void reset(void* object);
}

#endif