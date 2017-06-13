#include <cmath>

#include "gtest/gtest.h"
#include "lms/math/vertex.h"

namespace lms {
namespace math {

TEST(Vertex2, lengthSquared) {
    // vertex2f
    // simple case
    EXPECT_FLOAT_EQ(194, vertex2f(5, 13).lengthSquared());

    // zero vector
    EXPECT_FLOAT_EQ(0, vertex2f(0, 0).lengthSquared());

    // negative values
    EXPECT_FLOAT_EQ(229, vertex2f(-2, 15).lengthSquared());
    EXPECT_FLOAT_EQ(229, vertex2f(15, -2).lengthSquared());

    // vertex2i
    // simple case
    EXPECT_FLOAT_EQ(25, vertex2i(3, 4).lengthSquared());

    // zero vector
    EXPECT_FLOAT_EQ(0, vertex2i(0, 0).lengthSquared());

    // negative values
    EXPECT_FLOAT_EQ(169, vertex2i(-12, 5).lengthSquared());
    EXPECT_FLOAT_EQ(25, vertex2i(3, -4).lengthSquared());
}

TEST(Vertex2, angle) {
    // all four directions
    EXPECT_FLOAT_EQ(0, vertex2f(1, 0).angle());
    EXPECT_FLOAT_EQ(M_PI_2, vertex2f(0, 1).angle());
    EXPECT_FLOAT_EQ(M_PI, vertex2f(-1, 0).angle());
    EXPECT_FLOAT_EQ(-M_PI_2, vertex2f(0, -1).angle());

    EXPECT_FLOAT_EQ(M_PI_4, vertex2f(1, 1).angle());
}

TEST(Vertex2, add) {
    // operator +
    EXPECT_EQ(vertex2f(6, 8), vertex2f(-2, 4) + vertex2f(8, 4));

    // operator +=
    vertex2f a(8, 4);
    vertex2f b(-3, 15);
    EXPECT_EQ(vertex2f(5, 19), a += b);
    EXPECT_EQ(vertex2f(5, 19), a);
    EXPECT_EQ(vertex2f(-3, 15), b);
}

TEST(Vertex2, sub) {
    // operator -
    EXPECT_EQ(vertex2f(-10, 0), vertex2f(-2, 4) - vertex2f(8, 4));

    // operator -=
    vertex2f a(8, 4);
    vertex2f b(-3, 15);
    EXPECT_EQ(vertex2f(11, -11), a -= b);
    EXPECT_EQ(vertex2f(11, -11), a);
    EXPECT_EQ(vertex2f(-3, 15), b);
}

TEST(Vertex2, explicitConversion) {
    // from 2i to 2f
    EXPECT_EQ(vertex2f(2, 3), vertex2f(vertex2i(2, 3)));

    // from 2f to 2i
    EXPECT_EQ(vertex2i(2, 3), vertex2i(vertex2f(2, 3)));
    EXPECT_EQ(vertex2i(2, 3), vertex2i(vertex2f(2.1, 3.9)));
}

TEST(Vertex2, mul) {
    // operator *
    EXPECT_EQ(vertex2f(-6, 12), vertex2f(-2, 4) * 3);

    // operator *=
    vertex2f a(8, 1);
    EXPECT_EQ(vertex2f(24, 3), a *= 3);
    EXPECT_EQ(vertex2f(24, 3), a);
}

TEST(Vertex2, div) {
    // operator *
    EXPECT_EQ(vertex2f(-4, 1), vertex2f(-12, 3) / 3);

    // operator *=
    vertex2f a(24, -4);
    EXPECT_EQ(vertex2f(12, -2), a /= 2);
    EXPECT_EQ(vertex2f(12, -2), a);
}

TEST(Vertex2, normalize) {
    EXPECT_EQ(vertex2f(12.f / 13, 5.f / 13), vertex2f(12, 5).normalize());
}

TEST(Vertex2, negate) {
    EXPECT_EQ(vertex2f(-3, 5), vertex2f(3, -5).negate());
    EXPECT_EQ(vertex2f(1, 2), -vertex2f(-1, -2));
}

TEST(Vertex2, rotate) {
    EXPECT_EQ(vertex2i(0, -1), vertex2i(1, 0).rotateClockwise90deg());
    EXPECT_EQ(vertex2i(1, -1), vertex2i(1, 1).rotateClockwise90deg());
    EXPECT_EQ(vertex2i(0, 0), vertex2i(0, 0).rotateClockwise90deg());

    EXPECT_EQ(vertex2i(0, 1), vertex2i(1, 0).rotateAntiClockwise90deg());
    EXPECT_EQ(vertex2i(-1, 1), vertex2i(1, 1).rotateAntiClockwise90deg());
    EXPECT_EQ(vertex2i(0, 0), vertex2i(0, 0).rotateAntiClockwise90deg());
}

TEST(VERTEX2, minimum_distance){
    float onTheSegment,minDistance;
    minDistance = minimum_distance(lms::math::vertex2f(0,0),lms::math::vertex2f(1,0),lms::math::vertex2f(0.5,0.5),onTheSegment);
    EXPECT_FLOAT_EQ(0.5,minDistance);
    EXPECT_FLOAT_EQ(0.5,onTheSegment);
}

// As one can see if we go clockwise with points on the border of
// the triangle we get false results. Ergo always go anticlockwise.
TEST(pointInTriangle, ClockwiseOnBorder) {
    {
        vertex2f p(1.5, 1.5);
        vertex2f v1(1, 1);
        vertex2f v2(1, 2);
        vertex2f v3(2, 2);
        vertex2f v4(2, 1);

        EXPECT_EQ(pointInTriangle(p, v1, v2, v3), false);
        EXPECT_EQ(pointInTriangle(p, v1, v3, v4), false);
    }

    {
        vertex2f p1(1, 1.5);
        vertex2f p2(1.5, 1);
        vertex2f v1(1, 1);
        vertex2f v2(1, 2);
        vertex2f v3(2, 2);
        vertex2f v4(2, 1);

        EXPECT_EQ(pointInTriangle(p1, v1, v2, v3), false);
        EXPECT_EQ(pointInTriangle(p2, v1, v3, v4), false);
    }

    {
        vertex2f p1(1.5, 2);
        vertex2f p2(2, 1.5);
        vertex2f v1(1, 1);
        vertex2f v2(1, 2);
        vertex2f v3(2, 2);
        vertex2f v4(2, 1);

        EXPECT_EQ(pointInTriangle(p1, v1, v2, v3), false);
        EXPECT_EQ(pointInTriangle(p2, v1, v3, v4), false);
    }
}

TEST(pointInTriangle, AntiClockwiseOnBorder) {
    {
        vertex2f p(1.5, 1.5);
        vertex2f v1(1, 1);
        vertex2f v2(2, 1);
        vertex2f v3(2, 2);
        vertex2f v4(1, 2);

        EXPECT_EQ(pointInTriangle(p, v1, v2, v3), true);
        EXPECT_EQ(pointInTriangle(p, v1, v3, v4), true);
    }

    {
        vertex2f p1(1.5, 1);
        vertex2f p2(1, 1.5);
        vertex2f v1(1, 1);
        vertex2f v2(2, 1);
        vertex2f v3(2, 2);
        vertex2f v4(1, 2);

        EXPECT_EQ(pointInTriangle(p1, v1, v2, v3), true);
        EXPECT_EQ(pointInTriangle(p2, v1, v3, v4), true);
    }

    {
        vertex2f p1(2, 1.5);
        vertex2f p2(1.5, 2);
        vertex2f v1(1, 1);
        vertex2f v2(2, 1);
        vertex2f v3(2, 2);
        vertex2f v4(1, 2);

        EXPECT_EQ(pointInTriangle(p1, v1, v2, v3), true);
        EXPECT_EQ(pointInTriangle(p2, v1, v3, v4), true);
    }

    {
        vertex2f p(1.5, -1.5);
        vertex2f v1(1, -1);
        vertex2f v2(1, -2);
        vertex2f v3(2, -2);
        vertex2f v4(2, -1);

        EXPECT_EQ(pointInTriangle(p, v1, v2, v3), true);
        EXPECT_EQ(pointInTriangle(p, v1, v3, v4), true);
    }
}

TEST(pointInTriangle, AntiClockwiseNotOnBorder) {
    {
        vertex2f p1(1.75, 1.25);
        vertex2f p2(1.25, 1.75);
        vertex2f v1(1, 1);
        vertex2f v2(2, 1);
        vertex2f v3(2, 2);
        vertex2f v4(1, 2);

        EXPECT_EQ(pointInTriangle(p1, v1, v2, v3), true);
        EXPECT_EQ(pointInTriangle(p2, v1, v3, v4), true);
    }

    {
        vertex2f p1(1.25, -1.75);
        vertex2f p2(1.75, -1.25);
        vertex2f v1(1, -1);
        vertex2f v2(1, -2);
        vertex2f v3(2, -2);
        vertex2f v4(2, -1);

        EXPECT_EQ(pointInTriangle(p1, v1, v2, v3), true);
        EXPECT_EQ(pointInTriangle(p2, v1, v3, v4), true);
    }

    {
        vertex2f p1(1.25, 1.75);
        vertex2f p2(1.75, 1.25);
        vertex2f v1(1, 1);
        vertex2f v2(2, 1);
        vertex2f v3(2, 2);
        vertex2f v4(1, 2);

        EXPECT_EQ(pointInTriangle(p1, v1, v2, v3), false);
        EXPECT_EQ(pointInTriangle(p2, v1, v3, v4), false);
    }

    {
        vertex2f p1(1.75, -1.25);
        vertex2f p2(1.25, -1.75);
        vertex2f v1(1, -1);
        vertex2f v2(1, -2);
        vertex2f v3(2, -2);
        vertex2f v4(2, -1);

        EXPECT_EQ(pointInTriangle(p1, v1, v2, v3), false);
        EXPECT_EQ(pointInTriangle(p2, v1, v3, v4), false);
    }
}

// This test case is just here to showcase the problems that might
// arise with floating point values that are very close or on the edges.
TEST(pointInTriangle, AntiClockWiseSmallFloatsAroundEdges) {
    {
        vertex2f v1(0.1345, 0.8672);
        vertex2f v2(0.2165, 0.9687);
        vertex2f v3(0.1923, 1.1687);
        vertex2f v4(0.1484, 1.0672);
        // Get the midpoint between v1 and v3.
        vertex2f p((v1.x + v3.x) / 2, (v1.y + v3.y) / 2);

        EXPECT_EQ(pointInTriangle(p, v1, v2, v3), false) << p.x << "," << p.y;
        EXPECT_EQ(pointInTriangle(p, v1, v3, v4), true) << p.x << "," << p.y;
    }

    {
        vertex2f v1(0.1345, 0.8672);
        vertex2f v2(0.2165, 0.9687);
        vertex2f v3(0.1923, 1.1687);
        vertex2f v4(0.1484, 1.0672);
        // Get the midpoint between v1 and v3 and add some y sugar.
        vertex2f p((v1.x + v3.x) / 2, (v1.y + v3.y + 0.00001) / 2);

        EXPECT_EQ(pointInTriangle(p, v1, v2, v3), false) << p.x << "," << p.y;
        EXPECT_EQ(pointInTriangle(p, v1, v3, v4), true) << p.x << "," << p.y;
    }
}

} // namespace math
} // namespace lms
