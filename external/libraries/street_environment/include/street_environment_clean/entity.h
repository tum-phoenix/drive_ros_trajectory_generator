#pragma once

#include "lms/math/pose.h"
#include <memory>
#include "bounding_box.h"
//has a pose2D

namespace lms{
namespace street_environment{
class Entity{
    static constexpr int TYPE = 0;
    lms::math::Pose2D m_pose;
public:
    void pose(lms::math::Pose2D pose){
        this->m_pose = pose;
    }
    lms::math::Pose2D pose(){
        return m_pose;
    }
};
typedef std::shared_ptr<Entity> EntityPtr;

class Obstacle:public Entity{
    /**
     * @brief boundingBox local coordinates around the centre of mass
     */
    BoundingBox2f boundingBox;
    std::vector<Eigen::Vector2f> points;

public:
    void addPoints(std::vector<Eigen::Vector2f> points){
        //TODO

        //First way
        //add position to all current points
        //add points
        //calculate center of mass
        //substract center of mass from all points
        //recalculate bounding box

        //Second way
        //just add the points as the pose is not the center of mass
    }
};

class Sign:public Entity{
    enum class Type{
        UNKNOWN,STOP,GIVE_WAY_SIGN, SPEED_LIMIT,OVERTAKING_FORBIDDEN,OVERTAKING_ALLOWED,
    };
    float speedLimit;
};

}
}
