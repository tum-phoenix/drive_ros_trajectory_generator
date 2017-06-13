#ifndef STREET_ENVIRONMENT_BASIC_OBSTACLE_H
#define STREET_ENVIRONMENT_BASIC_OBSTACLE_H

#include <vector>

#include <lms/math/vertex.h>
#include <lms/serializable.h>
#include <street_environment/bounding_box.h>
#include <cereal/types/vector.hpp>

namespace street_environment {

class BasicObstacle : public lms::Serializable {
   public:
    BasicObstacle() = default;
    BasicObstacle(const std::vector<lms::math::vertex2f>& points)
        : m_points(points) {}
    virtual ~BasicObstacle() = default;

    BasicObstacle(const BasicObstacle&) = default;
    BasicObstacle& operator=(const BasicObstacle&) = default;
    BasicObstacle(BasicObstacle&&) = default;
    BasicObstacle& operator=(BasicObstacle&&) = default;

    std::vector<lms::math::vertex2f>& points() { return m_points; }
    const std::vector<lms::math::vertex2f>& points() const { return m_points; }

    void translate(const lms::math::vertex2f& vector);
    void rotate(float radians);

    street_environment::BoundingBox2f boundingBox() const;

    /////////////////////////////// Serialization //////////////////////////////
    CEREAL_SERIALIZATION()

    template <class Archive>
    void serialize(Archive& archive) {
        archive(m_points);
    }

   private:
    std::vector<lms::math::vertex2f> m_points;
};

using BasicObstacleVector = std::vector<BasicObstacle>;

}  // namespace street_environment

#endif  // STREET_ENVIRONMENT_BASIC_OBSTACLE_H
