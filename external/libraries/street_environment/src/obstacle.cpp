#include "street_environment/obstacle.h"

#include "lib/objectTracker/objectTracker.h"
#include "lib/objectTracker/objectTracker_emxAPI.h"

namespace street_environment{
Obstacle::Obstacle() : m_position(0, 0){
    invalid();//needed as it has to be invalid if it is deserialized
}


bool Obstacle::match(const Obstacle &obj) const{
    if(!EnvironmentObject::match(obj)){
        return false;
    }

    return (position().distance(obj.position())<0.3);
}

lms::math::vertex2f Obstacle::position() const{
    validate();
    return m_position;
}

} //street_environment
