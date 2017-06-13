#ifndef STREET_ENVIRONMENT_CAR_H
#define STREET_ENVIRONMENT_CAR_H
#include "street_environment/dynamic_entity.h"
#include "lms/time.h"

#include "cereal/types/vector.hpp"
#include "cereal/types/string.hpp"
#include "cereal/types/base_class.hpp"
#include "cereal/cerealizable.h"
#include "lms/serializable.h"
#include "cereal/cerealizable.h"
#include "cereal/cereal.hpp"
#include "cereal/types/polymorphic.hpp"
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/base_class.hpp"

namespace street_environment {
/**
 * TODO Add hasState()
 * @brief The Car command class
 */
class CarCommand:public DynamicEntity{
public:
    CarCommand(){
    }
    enum class StateType{
        NOT_DEFINED,IDLE,DRIVING,PARKING,RACE
    };
    struct State
        : public lms::Serializable
    {
        State():priority(0),state(StateType::NOT_DEFINED),indicatorLeft(false),indicatorRight(false),startState(lms::Time::ZERO),
            endState(lms::Time::ZERO),steering_front(0),steering_rear(0),targetSpeed(0),targetDistance(0){}

        /**
         * @brief priority of the state, for example StateType::DRIVING could have priority 1, PARKING could have 2 -> the car would par
         */
        int priority;
        std::string name;
        StateType state;
        bool indicatorLeft;
        bool indicatorRight;

        /**
         * @brief startState
         */
        lms::Time startState;
        /**
         * @brief endState time till the car should do something (for example idle)
         */
        lms::Time endState;
        float steering_front, steering_rear;
        float targetSpeed;
        /**
         * @brief targetDistance distance that should have the following speed, used to stop the car at crossings
         */
        float targetDistance;
        /**
         * @brief intime
         * @param currentTime
         * @return
         */
        bool intime(const lms::Time &currentTime) const{
            if(endState == lms::Time::ZERO && startState == lms::Time::ZERO){
                return true;
            }
            return startState < currentTime && currentTime < endState;
        }

    CEREAL_SERIALIZATION()

        template <class Archive>
        void serialize( Archive & archive) {
            archive(priority, name, state, startState, endState, steering_front,
                    steering_rear, targetSpeed);
        }
    };
private:
    std::vector<State> states; //TODO not sure if it should be public
public:

    State getPrioState() const{
        if(states.size() == 0){
            //TODO throw error
            return State();
        }
        State prio = states[0];
        for(uint i = 1; i < states.size();i++){
            if(states[i].priority > prio.priority)
                prio = states[i];
        }
        return prio;
    }

    float steeringFront() const{
        return getPrioState().steering_front;
    }

    float steeringRear() const{
        return getPrioState().steering_rear;
    }

    float targetSpeed() const{
        return getPrioState().targetSpeed;
    }

    void addState(const State &s){
        states.push_back(s);
    }

    void putState(const State &s){
        for(uint i = 0; i < states.size(); i++){
            if(states[i].name == s.name){
                states[i] = s;
                return;
            }
        }
        addState(s);
    }

    bool removeState(const std::string &name){
        for(uint i = 0; i < states.size();){
            if(states[i].name == name){
                states.erase(states.begin()+i);
                return true;
            }else{
                i++;
            }
        }
        return false;
    }

    State* getState(const std::string &name){
        for(State &s:states){
            if(s.name == name)
                return &s;
        }
        return nullptr;
    }

    /**
     * @brief validateStates removed invalid/outdated states
     * @param currentTime
     */
    void validateStates(const lms::Time &currentTime){
        for(uint i = 0; i < states.size();){
            if(!states[i].intime(currentTime)){
                states.erase(states.begin()+i);
            }else{
                i++;
            }
        }
    }

    // cereal implementation
    //get default interface for datamanager
    CEREAL_SERIALIZATION()

    template <class Archive>
    void serialize( Archive & archive) {
        //TODO
        archive(states);
        archive(cereal::base_class<DynamicEntity>(this));
    }
};
}//street_environment

#endif //STREET_ENVIRONMENT_CAR_H
