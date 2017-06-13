#ifndef STREET_ENVIRONMENT_START_LINE_H
#define STREET_ENVIRONMENT_START_LINE_H
#include "obstacle.h"
namespace street_environment{
class StartLine:public Obstacle{
public:
    static constexpr int TYPE = 3;

    int getType() const override{
       return TYPE;
    }


    //get default interface for datamanager
    CEREAL_SERIALIZATION()

    template<class Archive>
    void serialize(Archive & archive) {
        archive (
                cereal::base_class<street_environment::Obstacle>(this));
    }
};
typedef std::shared_ptr<StartLine> StartLinePtr;
} //namespace street_environment

#endif //STREET_ENVIRONMENT_START_LINE_H
