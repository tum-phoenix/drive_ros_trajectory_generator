#ifndef LMS_STREET_ENVIRONMENT_CROSSING_H
#define LMS_STREET_ENVIRONMENT_CROSSING_H
#include "obstacle.h"
#include "lms/time.h"

namespace street_environment{
/**
 * @brief A dynamic entity can be the vehicle itself but also every other
 * moving obstacle.
 */
class Crossing:public Obstacle
{
private:
    bool m_blocked;
    lms::Time m_startStop;
    float m_stopTime;

public:
    static constexpr int TYPE = 2;
    virtual int getType() const override{
       return Crossing::TYPE;
    }
    bool foundOppositeStopLine; //not that nice
    bool startStop(){
        if(m_startStop == lms::Time::ZERO){
            m_startStop = lms::Time::now();
            return true;
        }
        return false;
    }
    bool hasToStop() const{
        if(m_startStop != lms::Time::ZERO){
            return m_startStop.since().toFloat() < m_stopTime; //Time that we have to wait for an obstacle magic number
        }
        return true;
    }

    lms::Time stopTime(){
        return m_startStop;
    }

    bool hasWaited(){
        return m_startStop != lms::Time::ZERO;
    }

    Crossing():m_blocked(false),m_startStop(lms::Time::ZERO),m_stopTime(3),foundOppositeStopLine(0){
    }

    virtual bool match(const Crossing &obj) const{
        if(!Obstacle::match(obj)){
            return false;
        }
        //TODO
        return false;
    }

    void blocked(bool blocked){
        m_blocked = blocked;
    }

    lms::math::Rect blockedRect() const{
        lms::math::vertex2f crossingPos = position();
        lms::math::vertex2f crossingView = viewDirection().normalize();
        lms::math::Rect blockedRect;
        lms::math::vertex2f pos = crossingPos+crossingView*0.4+crossingView.rotateClockwise90deg()*0.8;
        blockedRect.x = pos.x;
        blockedRect.y = pos.y;
        blockedRect.width = 0.4;
        blockedRect.height = 1.4;
        return blockedRect;
    }

    bool blocked() const{
        return m_blocked;
    }

    // cereal implementation
        //get default interface for datamanager
        CEREAL_SERIALIZATION()

        template<class Archive>
        void serialize(Archive & archive) {
            archive (
                cereal::base_class<street_environment::Obstacle>(this),
                m_blocked, m_startStop);
        }

};

typedef std::shared_ptr<Crossing> CrossingPtr;
}//street_environment
#endif //LMS_STREET_ENVIRONMENT_CROSSING_H
