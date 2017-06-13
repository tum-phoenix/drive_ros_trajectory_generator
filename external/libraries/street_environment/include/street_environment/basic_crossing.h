#ifndef STREET_ENVIRONMENT_BASIC_OBSTACLE_H
#define STREET_ENVIRONMENT_BASIC_OBSTACLE_H

#include <vector>

#include <lms/math/vertex.h>
#include <lms/serializable.h>
#include <street_environment/bounding_box.h>
#include <cereal/types/vector.hpp>
#include <lms/math/pose.h>

namespace street_environment {

class BasicCrossing : public lms::Serializable {
   public:
    /**
     * @brief pose middle of the crossing
     */
    lms::math::Pose2D pose;
    BasicCrossing() = default;
    virtual ~BasicCrossing() = default;

    street_environment::BoundingBox2f boundingBox() const;

    /////////////////////////////// Serialization //////////////////////////////
    CEREAL_SERIALIZATION()

    template <class Archive>
    void serialize(Archive& archive) {
        archive(pose);
    }

   private:
    std::vector<lms::math::vertex2f> m_points;
};

using BasicObstacleVector = std::vector<BasicObstacle>;

}  // namespace street_environment

#endif  // STREET_ENVIRONMENT_BASIC_OBSTACLE_H
