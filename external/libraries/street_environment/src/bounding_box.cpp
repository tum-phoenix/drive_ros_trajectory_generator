#include "street_environment/bounding_box.h"

namespace street_environment {

template <typename V>
BoundingBox<V>::BoundingBox(const std::vector<V>& points) {
    float minX = points.at(0).x;
    float maxX = minX;
    float minY = points.at(0).y;
    float maxY = minY;

    for (const auto& point : points) {
        if (point.x < minX) {
            minX = point.x;
        }
        if (point.x > maxX) {
            maxX = point.x;
        }
        if (point.y < minY) {
            minY = point.y;
        }
        if (point.y > maxY) {
            maxY = point.y;
        }
    }

    m_corners[0] = V(minX, minY);
    m_corners[1] = V(maxX, minY);
    m_corners[2] = V(maxX, maxY);
    m_corners[3] = V(minX, maxY);
}

template <typename V>
BoundingBox<V>::BoundingBox(const lms::math::PointCloud<V>& pointCloud)
    : BoundingBox(pointCloud.points()) {}

template class BoundingBox<lms::math::vertex2f>;

}  // namespace street_environment
