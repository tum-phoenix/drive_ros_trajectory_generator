#ifndef STREET_ENVIRONMENT_DYNAMIC_ENTITY_H
#define STREET_ENVIRONMENT_DYNAMIC_ENTITY_H

#include "lms/math/vertex.h"
namespace street_environment {
/**
 * @brief A dynamic entity can be the vehicle itself but also every other
 * moving obstacle.
 */
class DynamicEntity : public lms::Serializable
{
protected:
    /**
     * @brief Global position of the entity. x and y are given in meters.
     */
    lms::math::vertex2f m_position, m_lastposition;

    /**
     * @brief Direction vector that points to the direction the entity is
     * looking at.
     */
    lms::math::vertex2f m_viewDirection,m_lastviewDirection;

    /**
     * @brief Velocity in m/s of the entity.
     */
    float m_velocity;

    /**
     * @brief Turn rate (rad/s) of the entity
     */
    float m_turnRate;

    /**
     * @brief Direction vector that points to the direction the entity is
     * driving to.
     */
    lms::math::vertex2f moveDirection;

public:
    DynamicEntity();
    virtual ~DynamicEntity(){}

    lms::math::vertex2f viewDirection() const{
        return m_viewDirection;
    }

    lms::math::vertex2f position()const{
        return m_position;
    }

    float velocity()const{
        return m_velocity;
    }

    float turnRate()const{
        return m_turnRate;
    }

    /**
     * @brief Set the position for the current cycle. Should be called only
     * once per cycle.
     * @param position global position of the entity
     * @param viewDirection direction vector of the view of the entity
     */
    void updatePosition(const lms::math::vertex2f &m_position,
                        const lms::math::vertex2f &m_viewDirection);

    //void updatePosition(float dx, float dy, float dphi);

    /**
     * @brief Set the velocity for the current cycle. Should be called only
     * once per cycle.
     * @param velocity velocity of the entity
     * @param moveDirection direction vector of the movement of the entity
     */
    void updateVelocity(float velocity,
                        const lms::math::vertex2f &moveDirection);


    /**
    * @brief Set the turn rate for the current cycle. Should be called only
    * once per cycle.
    * @param turnRate The turn rate of the entity
    */
    void updateTurnRate(float turnRate);

    //####Delta-Values

    lms::math::vertex2f deltaPosition() const;
    lms::math::vertex2f localDeltaPosition() const;

    /**
     * @brief Difference in velocity to the last cycle.
     * @return delta velocity in m/s
     */
    float deltaPhi() const;
    float movedDistance() const;


    // cereal implementation
    //get default interface for datamanager
    CEREAL_SERIALIZATION()

    template <class Archive>
    void serialize( Archive & archive) {
        archive(m_position,m_velocity,m_viewDirection);
    }
};
}//sensor_utils

#endif /* STREET_ENVIRONMENT_DYNAMIC_ENTITY_H */
