#include <algorithm>
#include <vector>
#include <tuple>
#include <map>
#include <set>
#include <random>
#include <cmath>
#include <cstdio>

struct QTable {
    typedef std::tuple<unsigned, float, float> StateSetting;
    typedef std::vector<StateSetting> StateSettings;
    typedef std::tuple<unsigned, float, float> ActionsSettings;
    typedef std::vector<unsigned> State;
    typedef std::vector<float> ActionSpace; // Contains rewards

    const StateSettings state_settings;
    const ActionsSettings action_settings;
    std::map<State, ActionSpace> table;
    float last_reward = 0;
    std::set<std::vector<unsigned>> explored;

    unsigned max_states = 1;
    unsigned action_count = 0;

    float learning_rate;
    float epsilon;
    float discount_factor;

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
        std::map<State, ActionSpace>::iterator action_space_it = table.find(state);
        bool new_state = false;
        if (action_space_it == table.end()){
            new_state = true;
            action_space_it = table.emplace(std::piecewise_construct,
                    std::make_tuple(state),
                    std::make_tuple(action_count, std::numeric_limits<float>::quiet_NaN())).first;
        }

        ActionSpace &action_space = action_space_it->second;
        static std::mt19937 random_generator{std::random_device{}()};
        static const std::uniform_real<float> real_distr(0, 1);
        static const std::uniform_int_distribution<unsigned> random_action(0, action_count-1);
        unsigned action;
        if (new_state || real_distr(random_generator) < epsilon){ // Choose random action
            action = random_action(random_generator);
        }
        else { // Randomly choose the max value if it isn't nan
            ActionSpace::const_iterator &action_it = std::max_element(action_space.begin(), action_space.end());
            const float max_value = *action_it;
            if (std::isnan(max_value)){
                action = random_action(random_generator);
            }
            else {
                action = action_it - action_space.begin();
            }
        }
        std::vector<unsigned> full_state = state;
        full_state.push_back(action);
        explored.insert(full_state);
        return {raw_action(action), action_space.begin()+action};
    }

    float update_action(ActionSpace::iterator &last_action, const float &reward, const State &state){
        std::map<State, ActionSpace>::iterator action_space_it = table.find(state);
        float max_q = 0;
        if (action_space_it != table.end()){
            const ActionSpace &action_space = action_space_it->second;
            ActionSpace::const_iterator &action_it = std::max_element(action_space.begin(), action_space.end());
            max_q = std::isnan(*action_it) ? 0 : *action_it;
        }
        *last_action = std::isnan(*last_action) ? 0 : *last_action;
        *last_action += learning_rate*(reward + discount_factor*max_q - *last_action);
        last_reward = *last_action;
        return *last_action;
    }

    inline const State get_state(const std::vector<float> &raw_state){
        State state(state_settings.size());
        State::iterator value = state.begin();
        StateSettings::const_iterator setting = state_settings.begin();
        for (std::vector<float>::const_iterator raw_value = raw_state.begin(); raw_value != raw_state.end(); raw_value++){
            const float normalized_value = fminf(1, fmaxf(0, (*raw_value-std::get<1>(*setting))
                    / (std::get<2>(*setting)-std::get<1>(*setting))));
            *value = static_cast<unsigned>(normalized_value*std::get<0>(*setting)*0.9999);
            value++;
            setting++;
        }

        return state;
    }

    inline float raw_action(const unsigned action){
        static const float offset = action_count%2 == 0 ? 0 :
                2.f/static_cast<float>(std::get<0>(action_settings))
                *(std::get<2>(action_settings)-std::get<1>(action_settings))+std::get<1>(action_settings);
        const float normalized_value = action / static_cast<float>(std::get<0>(action_settings));
        return normalized_value*(std::get<2>(action_settings)-std::get<1>(action_settings))
            + std::get<1>(action_settings) - offset;
    }

};