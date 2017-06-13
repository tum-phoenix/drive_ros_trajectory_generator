#include <street_environment/dynamic_entity.h>

namespace street_environment {

DynamicEntity::DynamicEntity() : m_position(0, 0),m_lastposition(0,0), m_viewDirection(1, 0),m_lastviewDirection(1,0),
    m_velocity(0), moveDirection(1, 0) {}

void DynamicEntity::updatePosition(const lms::math::vertex2f &position,
                                   const lms::math::vertex2f &viewDirection) {
    this->m_lastposition = this->m_position;
    this->m_position = position;
    m_lastviewDirection = m_viewDirection;
    this->m_viewDirection = viewDirection;
}

void DynamicEntity::updateVelocity(float velocity,
                    const lms::math::vertex2f &moveDirection) {
    this->m_velocity = velocity;
    this->moveDirection = moveDirection;
}

float DynamicEntity::movedDistance() const {
    return m_lastposition.distance(m_position);
}

lms::math::vertex2f DynamicEntity::deltaPosition() const{
    return m_position-m_lastposition;
}

lms::math::vertex2f DynamicEntity::localDeltaPosition() const  {
    return deltaPosition().rotate(-m_lastviewDirection.angle()); //rotate it clockwise
}

float DynamicEntity::deltaPhi() const{
    return m_viewDirection.angleBetweenWithOrientation(m_lastviewDirection); //TODO check if this works!

}

void DynamicEntity::updateTurnRate(float turnRate){
    //this->lastTurnRate = this->m_turnRate;
    this->m_turnRate = turnRate;
}

}//sensor_utils
