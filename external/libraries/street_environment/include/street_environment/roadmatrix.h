#ifndef STREET_ENVIRONMENT_ROAD_MATRIX_H_
#define STREET_ENVIRONMENT_ROAD_MATRIX_H_

#include <array>
#include <ostream>
#include <vector>

#include <cereal/cerealizable.h>
#include <lms/serializable.h>
#include <cereal/types/array.hpp>
#include <cereal/types/vector.hpp>

#include "lms/math/polyline.h"
#include "lms/math/vertex.h"

namespace street_environment {

struct RoadMatrixCell : lms::Serializable {
    bool contains(const lms::math::vertex2f& p) const {
        return lms::math::pointInTriangle(p, points[0], points[1], points[2]) ||
               lms::math::pointInTriangle(p, points[0], points[2], points[3]);
    }

    // X and y coordinates of the cell in a RoadMatrix representation.
    int x, y;
    // counterclockwise
    std::array<lms::math::vertex2f, 4> points;
    bool hasObstacle = false;

    CEREAL_SERIALIZATION()

    template <class Archive>
    void serialize(Archive& archive) {
        archive(x, y, points, hasObstacle);
    }
};

/**
 * @brief RoadMatrix stores a cell representation of the road.
 */
class RoadMatrix : public lms::Serializable {
   public:
    /**
     * @brief Initializes the RoadMatrix with config values.
     * A RoadMatrix instance can only be initialized once during its lifetime.
     * Any further calls to initialize are disregarded.
     */
    bool initialize(float laneWidth, int cellsPerLane, float cellLength,
                    float maxTranslation);

    /**
     * @brief Creates a RoadMatrix representation around "line".
     * @param line: A line that the roadmatrix is structured around. This should
     * normally be the center line.
     */
    void aroundLine(const lms::math::polyLine2f& line);

    /**
     * @brief Tries to find a cell that contains v.
     * Returns false if no cell is found.
     * Returns true and sets 'foundCell' to a copy of the found cell if a cell
     * is found. If v is on the edge of two or more cells 'foundCell' will be
     * the first first(smallest x, smallest y) of those cells.
     */
    bool findCell(const lms::math::vertex2f& v,
                  street_environment::RoadMatrixCell* foundCell);

    const RoadMatrixCell& cell(int x, int y) const;
    RoadMatrixCell& cell(int x, int y);

    int width() const { return m_width; }
    int length() const { return m_length; }
    int size() const { return m_width * m_length; }

    float cellWidth() const { return m_cellWidth; }
    float cellLength() const { return m_cellLength; }

    int zeroColumn() const { return m_translation; }
    int lengthAhead() const { return length() - m_translation; }

    friend std::ostream& operator<<(std::ostream& stream,
                                    const RoadMatrix& matrix);
    CEREAL_SERIALIZATION()

    template <class Archive>
    void serialize(Archive& archive) {
        archive(m_cells, m_width, m_length, m_translation);
    }

    lms::math::polyLine2f m_negativeCenterLine;

   private:
    lms::math::polyLine2f prepCenterLine(const lms::math::polyLine2f& line);
    lms::math::polyLine2f negativeCenterLine(
        const lms::math::polyLine2f& line);

    void createMatrix(const lms::math::polyLine2f& line);

    const lms::math::vertex2f& point(int x, int y) const;
    lms::math::vertex2f& point(int x, int y);

    RoadMatrixCell initCell(int x, int y) const;
    void initCells();

    bool m_initalized = false;

    float m_cellWidth = 0;
    float m_cellLength = 0;
    int m_cellsPerLane = 0;

    int m_translation = 0;
    float m_maxTranslation = 0;

    // Width and length of the cell matrix.
    int m_width = 0;
    int m_length = 0;

    // Width and length of the point matrix.
    int m_pWidth = 0;
    int m_pLength = 0;

    std::vector<RoadMatrixCell> m_cells;
    std::vector<lms::math::vertex2f> m_points;
};

}  // namespace street_environment

#endif  // STREET_ENVIRONMENT_ROAD_MATRIX_H_
