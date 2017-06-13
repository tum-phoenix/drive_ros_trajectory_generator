#include "street_environment/basic_obstacle.h"

namespace street_environment {

void BasicObstacle::translate(const lms::math::vertex2f& vector) {
    for (auto& point : m_points) {
        point = point + vector;
    }
}

void BasicObstacle::rotate(float radians) {
    for (auto& point : m_points) {
        point = point.rotate(radians);
    }
}

street_environment::BoundingBox2f BasicObstacle::boundingBox() const {
    return street_environment::BoundingBox2f(m_points);
}

}  // namespace street_environment
