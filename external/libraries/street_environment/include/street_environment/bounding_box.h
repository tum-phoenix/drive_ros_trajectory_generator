#ifndef STREET_ENVIRONMENT_BOUNDING_BOX_H
#define STREET_ENVIRONMENT_BOUNDING_BOX_H

#include <array>
#include <vector>

#include <lms/math/point_cloud.h>
#include <lms/math/vertex.h>

namespace street_environment {

template <typename V>
class BoundingBox {
   public:
    BoundingBox() : m_corners() {}
    ~BoundingBox() {}
    BoundingBox(const std::vector<V>& points);
    BoundingBox(const lms::math::PointCloud<V>& pointCloud);

    const std::array<V, 4>& corners() const { return m_corners; }

   private:
    // Anticlockwise starting at minimum x minimum y
    std::array<V, 4> m_corners;
};

using BoundingBox2f = BoundingBox<lms::math::vertex2f>;
using BoundingBox2fVector = std::vector<BoundingBox2f>;

}  // namespace street_environment

#endif  // STREET_ENVIRONMENT_BOUNDING_BOX_H
