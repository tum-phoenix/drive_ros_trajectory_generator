#ifndef SENSOR_ENVIRONMENT_H
#define SENSOR_ENVIRONMENT_H
#include <vector>
#include <memory>
#include "lms/deprecated.h"

#include "lms/inheritance.h"
#include "lms/serializable.h"
#include "cereal/cerealizable.h"
#include "cereal/cereal.hpp"
#include "cereal/types/memory.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/string.hpp"
#include "cereal/access.hpp"
#include "cereal/types/polymorphic.hpp"
#include <set>

CEREAL_FORCE_DYNAMIC_INIT(street_environment)

namespace street_environment {
class EnvironmentObject :public virtual lms::Inheritance
{
private:
    std::string m_name;
    float m_trust;
    float m_trustLast;
    std::set<std::string> m_detectedBySensorId;
public:
    EnvironmentObject():m_trust(0),m_trustLast(0){
    }

    void addSensor(const std::string &sensorId){
        m_detectedBySensorId.insert(sensorId);
    }
    void addSensors(const std::set<std::string> &sensors){
        for(const std::string &s:sensors){
            addSensor(s);
        }
    }

    void removeSensor(const std::string &sensorId){
        m_detectedBySensorId.erase(sensorId);
    }
    bool detectedBySensor(const std::string &sensorId) const{
        return m_detectedBySensorId.count(sensorId);
    }
    bool detectedBySensors(const std::set<std::string> &sensorIds) const{
        int res = 0;
        for(const std::string &s:sensorIds){
            res += detectedBySensor(s);
        }
        return res;
    }
    std::set<std::string> sensors() const{
        return m_detectedBySensorId;
    }


    /**
     * @brief setTrust
     * @param trust value between 0 and 1
     */
    void setTrust(float trust){
        if(trust < 0 || trust > 1){
            throw "invalid trust: "+std::to_string(trust);
        }
        m_trustLast = m_trust;
        m_trust = trust;
    }

    float getLastTrust(){
        return m_trustLast;
    }

    float getDeltaTrust(){
        return m_trust - m_trustLast;
    }

    /**
     * @brief trust
     * @return
     */
    float trust() const{
        return m_trust;
    }

    virtual ~EnvironmentObject() {}

    virtual bool isSubType(std::type_index tIndex) const override{
        (void)tIndex;
        return false;
    }

    template <typename T>
    LMS_DEPRECATED std::shared_ptr<T> getCopyAsPtr() const{
        return std::shared_ptr<T>(new T(*static_cast<const T*>(this)));
    }


    template <typename T>
    LMS_DEPRECATED T& getAsReference() const{
        return *static_cast<T*>(this);
    }
    virtual int getType() const = 0;

    std::string name() const{
        return m_name;
    }
    void name(const std::string &name){
        m_name = name;
    }

    template <class Archive>
    void serialize( Archive & ar ) {
        ar(m_name, m_trust, m_trustLast);
    }

    virtual bool match(const EnvironmentObject &obj) const{
        return obj.getType() == getType();
    }
};

typedef std::shared_ptr<EnvironmentObject> EnvironmentObjectPtr;

template<typename T>
class Environment: public lms::Serializable{
public:
    virtual ~Environment() {}

    const std::shared_ptr<EnvironmentObject> getObjectByName(std::string name) const{
        for(const std::shared_ptr<EnvironmentObject> &o: objects){
            if(o->name() == name){
                return o;
            }
        }
        return nullptr;
    }
    std::vector<std::shared_ptr<T>> objects;

        //get default interface for datamanager
        CEREAL_SERIALIZATION()

        template<class Archive>
        void serialize(Archive &archive) {
            archive(objects);
        }
};
class RoadLane;
class Obstacle;
typedef Environment<EnvironmentObject> EnvironmentObjects;
typedef Environment<RoadLane> EnvironmentRoadLane;
typedef Environment<Obstacle> EnvironmentObstacles;
}  // namespace street_environment

namespace cereal {

template <class Archive, typename T>
struct specialize<Archive, street_environment::Environment<T>, cereal::specialization::member_serialize> {};
  // cereal no longer has any ambiguity when serializing street_environment::Environment

}  // namespace cereal


#endif //SENSOR_ENVIRONMENT_H
