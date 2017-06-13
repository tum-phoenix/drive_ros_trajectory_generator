#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "lms/math/vertex.h"
#include "street_environment/road.h"

#include "lms/serializable.h"
#include "cereal/cerealizable.h"
#include "cereal/cereal.hpp"
#include "cereal/types/polymorphic.hpp"
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/base_class.hpp"
#include "bounding_box.h"

namespace street_environment{
/**
 * @brief A dynamic entity can be the vehicle itself but also every other
 * moving obstacle.
 */
class Obstacle:public EnvironmentObject, public lms::Serializable{

    std::vector<lms::math::vertex2f> m_points;

    //Tmp variables
    mutable lms::math::vertex2f m_position;
    mutable bool valid;
    mutable BoundingBox2f m_boundingBox;

    void myValidate() const{
        calculatePosition();
        BoundingBox2f b(m_points);
        m_boundingBox = b;
        valid = true;
    }

    void calculatePosition() const{
        m_position.x = 0;
        m_position.y = 0;
        if(m_points.size() == 0){
            return;
        }
        for(const lms::math::vertex2f &v:m_points){
            m_position +=v;
        }
        m_position = m_position/m_points.size();
    }


public:
    static constexpr int TYPE = 1;
    virtual int getType() const override{
       return TYPE;
    }
    virtual ~Obstacle() {}

    virtual bool match(const Obstacle &obj) const;
    Obstacle();

    void invalid(){
        valid = false;
    }
    bool isvalid(){
        return valid;
    }
    void validate() const{
        if(!valid){
            this->myValidate();
        }
    }
    std::vector<lms::math::vertex2f> points() const{
        return m_points;
    }
    lms::math::vertex2f m_viewDirection = lms::math::vertex2f(1,0); //TODO
    void viewDirection(const lms::math::vertex2f &v){
        m_viewDirection = v;
    }

    lms::math::vertex2f viewDirection() const{
        return m_viewDirection;//TODO
    }

    void addPoint(const lms::math::vertex2f &v){
        invalid();
        m_points.push_back(v);
    }
    void addPoints(const std::vector<lms::math::vertex2f> &vs){
        for(const lms::math::vertex2f &v:vs){
            addPoint(v);
        }
    }
    void clearPoints(){
        m_points.clear();
    }


    lms::math::vertex2f position() const;
    //TODO add BoundingBox methods
    float m_width;
    float width() const{
        return m_width; //TODO
    }
    void width(float w){
        m_width = w;
    }

    void translate(float dx, float dy){
        invalid();
        lms::math::vertex2f delta(dx,dy);
        for(int i = 0; i < (int) m_points.size(); i++){
            m_points[i] = m_points[i]+delta;
        }
    }

    BoundingBox2f boundingBox() const{
        validate();
        return m_boundingBox;
    }

    template<class Archive>
    void serialize(Archive & archive) {
        archive (
            cereal::base_class<street_environment::EnvironmentObject>(this),m_points);
    }

    // cereal implementation
    //get default interface for datamanager
    CEREAL_SERIALIZATION()

};
typedef std::shared_ptr<Obstacle> ObstaclePtr;

} //street_environment

//CEREAL_REGISTER_TYPE(street_environment::Obstacle)

namespace cereal {
    template <class Archive>
    struct specialize<Archive, street_environment::Obstacle, cereal::specialization::member_serialize> {};
      // cereal no longer has any ambiguity when serializing street_environment::Obstacle
}  // namespace cereal

#endif /* OBSTACLE_H */
