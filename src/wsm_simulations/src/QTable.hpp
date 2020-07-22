#include <algorithm>
#include <vector>
#include <tuple>
#include <map>
#include <set>
#include <random>
#include <cmath>
#include <cstdio>
#include <fstream>

struct QTable {
    typedef std::tuple<unsigned, float, float> StateSetting;
    typedef std::vector<StateSetting> StateSettings;
    typedef std::tuple<unsigned, float, float> ActionsSettings;
    typedef std::vector<unsigned> State;
    typedef std::vector<float> ActionSpace; // Contains rewards

    StateSettings state_settings;
    ActionsSettings action_settings;
    std::map<State, ActionSpace> qtable;
    std::map<State, unsigned> explored; // This includes action (State+Action)
    unsigned long int update_count = 0;

    unsigned max_states = 1;
    unsigned action_count = 0;

    float learning_rate;
    float epsilon;
    float discount_factor;

    bool first = true;
    QTable::ActionSpace::iterator last_action;

    QTable(StateSettings state_settings, ActionsSettings action_settings,
            float learning_rate, float epsilon, float discount_factor):
        state_settings(state_settings), action_settings(action_settings),
        learning_rate(learning_rate), epsilon(epsilon), discount_factor(discount_factor){
        action_count = std::get<0>(action_settings);
        for (auto &stateSetting : state_settings){
            max_states *= std::get<0>(stateSetting);
        }
        max_states *= action_count;
    }

    inline std::tuple<float, ActionSpace::iterator> choose_action(const State state){
        static std::mt19937 random_generator{std::random_device{}()};
        static const std::uniform_real<float> real_distr(0, 1);
        static const std::uniform_int_distribution<unsigned> random_action(0, action_count-1);

        std::map<State, ActionSpace>::iterator action_space_it = qtable.find(state);
        bool new_state = false;
        if (action_space_it == qtable.end()){
            new_state = true;
            action_space_it = qtable.emplace(std::piecewise_construct,
                    std::make_tuple(state),
                    std::make_tuple(action_count, std::numeric_limits<float>::quiet_NaN())).first;
            for (auto &f : action_space_it->second){
                //f = real_distr(random_generator)*0.1f;
            }
        }

        ActionSpace &action_space = action_space_it->second;
        unsigned action;
        if (new_state || real_distr(random_generator) < epsilon){ // Choose random action
            action = random_action(random_generator);
        }
        else { // Randomly choose the max value if it isn't nan
            bool found_nonnan = false;
            unsigned max_ind; float max_value = -std::numeric_limits<float>::infinity();
            for (unsigned int i = 0; i < action_count; i++){
                if (!std::isnan(max_value) && max_value < action_space[i]){
                    found_nonnan = true;
                    max_ind = i; max_value = action_space[i];
                }
            }

            if (found_nonnan) {
                action = max_ind;
            }
            else {
                action = random_action(random_generator);
            }
        }
        std::vector<unsigned> full_state = state;
        full_state.push_back(action);
        if (!explored.insert(std::make_pair(full_state, 0)).second){
            explored.at(full_state) += 1;
        }

        /*printf("Full state: ");
        for (const auto &fs : full_state){
            printf("%d ", fs);
        }
        printf("\n");*/

        first = false;
        last_action = action_space.begin()+action;
        return {raw_action(action), action_space.begin()+action};
    }

    void reset(){
        first = false;
    }

    float update_action(const float &reward, const State &state){
        if (first){
            return std::numeric_limits<float>::quiet_NaN();
        }
        update_count++;
        std::map<State, ActionSpace>::iterator action_space_it = qtable.find(state);
        float max_q = 0;
        if (action_space_it != qtable.end()){
            const ActionSpace &action_space = action_space_it->second;
            ActionSpace::const_iterator &action_it = std::max_element(action_space.begin(), action_space.end());
            max_q = std::isnan(*action_it) ? 0 : *action_it;
        }
        *last_action = std::isnan(*last_action) ? 0 : *last_action;
        *last_action += learning_rate*(reward + discount_factor*max_q - *last_action);
        return *last_action;
    }

    inline const State get_state(const std::vector<float> &raw_state){
        State state(state_settings.size());
        State::iterator value = state.begin();
        StateSettings::const_iterator setting = state_settings.begin();
        for (std::vector<float>::const_iterator raw_value = raw_state.begin(); raw_value != raw_state.end(); raw_value++){
            const float normalized_value = fminf(1, fmaxf(0,
                    (*raw_value-std::get<1>(*setting)) / (std::get<2>(*setting)-std::get<1>(*setting))));
            *value = static_cast<unsigned>(normalized_value*std::get<0>(*setting)*0.9999);
            value++;
            setting++;
        }

        return state;
    }

    inline float raw_action(const unsigned action){
        const float normalized_value = (static_cast<float>(action)+0.5f) / static_cast<float>(std::get<0>(action_settings));
        return normalized_value*(std::get<2>(action_settings)-std::get<1>(action_settings))
            + std::get<1>(action_settings);
    }

    QTable &QTable::operator =(const QTable &other){
        state_settings = other.state_settings;
        action_settings = other.action_settings;
        qtable = other.qtable;
        explored = other.explored;
        update_count = other.update_count;
        max_states = other.max_states;
        action_count = other.action_count;
        learning_rate = other.learning_rate;
        epsilon = other.epsilon;
        discount_factor = other.discount_factor;
        return *this;
    }

    void write(std::ostream& f){
        // Write count
        f.write((char*) &update_count, sizeof(unsigned long int));

        // Write learning rate
        f.write((char*) &learning_rate, sizeof(float));

        // Write epsilon
        f.write((char*) &epsilon, sizeof(float));

        // Write discount factor
        f.write((char*) &discount_factor, sizeof(float));

        // Write state settings
        uint32_t state_count = state_settings.size();
        f.write((char*) &state_count, sizeof(uint32_t));
        for (const StateSetting &setting : state_settings){
            f.write((char*) &setting, sizeof(StateSetting));
        }

        // Write action settings
        f.write((char*) &action_settings, sizeof(ActionsSettings));

        // Write explored
        uint32_t explored_count = explored.size();
        f.write((char*) &explored_count, sizeof(uint32_t));
        for (const auto &state : explored){
            f.write((char*) state.first.data(), sizeof(unsigned)*(state_settings.size()+1)); // +1 because this includes the action
            f.write((char*) &state.second, sizeof(unsigned));
        }

        // Write qtable
        uint32_t qtable_entry_count = qtable.size();
        f.write((char*) &qtable_entry_count, sizeof(uint32_t));
        unsigned cc = 0;
        for (const auto &p : qtable){
            const State &state = p.first;
            const ActionSpace &action_space = p.second;
            f.write((char*) state.data(), state.size()*sizeof(unsigned));
            f.write((char*) action_space.data(), action_space.size()*sizeof(float));
            if (cc < 2000000000)
                printf("QT WRITE POSITION %d\n", int(f.tellp()));
            cc++;
        }
    }

    static QTable read(std::istream& f){
        // Read count
        unsigned long int update_count;
        f.read((char*) &update_count, sizeof(unsigned long int));

        // Read learning rate
        float learning_rate;
        f.read((char*) &learning_rate, sizeof(float));

        // Read epsilon
        float epsilon;
        f.read((char*) &epsilon, sizeof(float));

        // Read discount factor
        float discount_factor;
        f.read((char*) &discount_factor, sizeof(float));

        // Read state settings
        StateSettings state_settings;
        uint32_t state_count;
        f.read((char*) &state_count, sizeof(uint32_t));
        for (uint32_t i = 0; i < state_count; i++) {
            StateSetting setting;
            f.read((char*) &setting, sizeof(StateSetting));
            state_settings.push_back(setting);
        }

        // Read action settings
        ActionsSettings action_settings;
        f.read((char*) &action_settings, sizeof(ActionsSettings));

        // Instantiate qtable
        QTable qtable(state_settings, action_settings, learning_rate, epsilon, discount_factor);
        qtable.update_count = update_count;

        // Read explored
        std::map<State, unsigned> &explored = qtable.explored;
        uint32_t explored_count;
        f.read((char*) &explored_count, sizeof(uint32_t));
        for (uint32_t i = 0; i < explored_count; i++) {
            State state(state_settings.size()+1);
            for (unsigned j = 0; j < state_settings.size()+1; j++){
                f.read((char*) &state[j], sizeof(unsigned));
            }
            unsigned c;
            f.read((char*) &c, sizeof(unsigned));
            explored.insert(std::make_pair(state, c));
            if (int(f.tellg()) == -1){
                throw std::runtime_error("Error reading file. Make sure stream open in binary mode.");
            }
        }
        if (int(f.tellg()) == -1){
            throw std::runtime_error("Error reading file. Make sure stream open in binary mode.");
        }
        // Read qtable
        std::map<State, ActionSpace> &qt = qtable.qtable;
        uint32_t qtable_count;
        f.read((char*) &qtable_count, sizeof(uint32_t));
        for (uint32_t i = 0; i < qtable_count; i++){
            if (int(f.tellg()) == -1){
                throw std::runtime_error("Error reading file. Make sure stream open in binary mode.");
            }
            const State state(state_settings.size());
            for (unsigned j = 0; j < state_settings.size(); j++){
                f.read((char*) &state[j], sizeof(unsigned));
            }
            const ActionSpace action_space(std::get<0>(action_settings));
            for (unsigned j = 0; j < action_space.size(); j++){
                f.read((char*) &action_space[j], sizeof(float));
            }
            qt.insert(std::make_pair(state, action_space));

        }

        return qtable;
    }

    void print() const {
        // Print QTable settings
        printf("QTable:\n");
        printf("learning_rate %f\nepsilon %f\ndiscount factor %f\n", learning_rate, epsilon, discount_factor);
        printf("Actions: %d, ranging from %f to %f\n", std::get<0>(action_settings),
          std::get<1>(action_settings), std::get<2>(action_settings));
        printf("State configuration:\n");
        for (auto &state_setting : state_settings){
            printf("\t%d, ranging from %f to %f\n", std::get<0>(state_setting),
                  std::get<1>(state_setting), std::get<2>(state_setting));
        }
        printf("------------\n");
        printf("Updated %d times\n", update_count);
        printf("Explored states: %d/%d\n", explored.size(), max_states);
        printf("States in qtable %d/%d\n", qtable.size(), max_states/action_count);
        printf("------------------------\n\n");
        fflush(stdout);
    }

};