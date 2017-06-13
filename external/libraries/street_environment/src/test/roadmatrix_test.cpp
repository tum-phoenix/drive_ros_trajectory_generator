#include "street_environment/roadmatrix.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "lms/math/polyline.h"

using lms::math::polyLine2f;
using lms::math::vertex2f;

namespace street_environment {

TEST(RoadMatrix, aroundLine) {
    polyLine2f line;
    int lineLength = 20;
    for (int i = 0; i <= lineLength; i++) {
        line.points().push_back(vertex2f(i * 0.1, 0));
    }

    RoadMatrix roadMatrix;
    roadMatrix.initialize(0.1, 1, 0.1, 5);
    roadMatrix.aroundLine(line, vertex2f(0, 0), 0);

    EXPECT_EQ(roadMatrix.length(), 20);
    EXPECT_EQ(roadMatrix.width(), 2);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 0).points.at(0).x, 0.5);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 1).points.at(3).x, 0.5);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 1).points.at(3).y, 0.1);
}

TEST(RoadMatrix, TranslateCase1) {
    polyLine2f line;
    int lineLength = 10;
    for (int i = 0; i <= lineLength; i++) {
        line.points().push_back(vertex2f(i * 0.1, 0));
    }

    RoadMatrix roadMatrix;
    roadMatrix.initialize(0.1, 1, 0.1, 5);
    roadMatrix.aroundLine(line, vertex2f(0, 0), 0);
    EXPECT_EQ(roadMatrix.length(), 10);
    roadMatrix.aroundLine(line, vertex2f(0.45, 0), 0);
    EXPECT_EQ(roadMatrix.length(), 14);

    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 0).points.at(0).x, -0.45);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 1).points.at(3).x, -0.45);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(3, 0).points.at(0).x, -0.15);
    EXPECT_FLOAT_EQ(roadMatrix.cell(3, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(3, 1).points.at(3).x, -0.15);
    EXPECT_FLOAT_EQ(roadMatrix.cell(3, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 0).points.at(0).x, 0);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 1).points.at(3).x, 0);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(13, 0).points.at(0).x, 0.9);
    EXPECT_FLOAT_EQ(roadMatrix.cell(13, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(13, 1).points.at(3).x, 0.9);
    EXPECT_FLOAT_EQ(roadMatrix.cell(13, 1).points.at(3).y, 0.1);
}

TEST(RoadMatrix, TranslateCase2) {
    polyLine2f line;
    int lineLength = 10;
    for (int i = 0; i <= lineLength; i++) {
        line.points().push_back(vertex2f(i * 0.1, 0));
    }

    RoadMatrix roadMatrix;
    roadMatrix.initialize(0.1, 1, 0.1, 5);
    roadMatrix.aroundLine(line, vertex2f(0, 0), 0);
    EXPECT_EQ(roadMatrix.length(), 10);
    roadMatrix.aroundLine(line, vertex2f(0.85, 0), 0);
    EXPECT_EQ(roadMatrix.length(), 15);

    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 0).points.at(0).x, -0.55);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 1).points.at(3).x, -0.55);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 0).points.at(0).x, -0.15);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 1).points.at(3).x, -0.15);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 0).points.at(0).x, 0);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 1).points.at(3).x, 0);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 0).points.at(0).x, 0.9);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 1).points.at(3).x, 0.9);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 1).points.at(3).y, 0.1);
}

TEST(RoadMatrix, TranslateCase3) {
    polyLine2f line;
    int lineLength = 10;
    for (int i = 0; i <= lineLength; i++) {
        line.points().push_back(vertex2f(i * 0.1, 0));
    }

    RoadMatrix roadMatrix;
    roadMatrix.initialize(0.1, 1, 0.1, 5);
    roadMatrix.aroundLine(line, vertex2f(0, 0), 0);
    EXPECT_EQ(roadMatrix.length(), 10);
    roadMatrix.aroundLine(line, vertex2f(0.55, 0), 0);
    EXPECT_EQ(roadMatrix.length(), 15);
    roadMatrix.aroundLine(line, vertex2f(0.45, 0), 0);
    EXPECT_EQ(roadMatrix.length(), 15);

    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 0).points.at(0).x, -0.6);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 1).points.at(3).x, -0.6);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 0).points.at(0).x, -0.15);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 1).points.at(3).x, -0.15);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 0).points.at(0).x, 0);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 1).points.at(3).x, 0);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 0).points.at(0).x, 0.9);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 1).points.at(3).x, 0.9);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 1).points.at(3).y, 0.1);
}

TEST(RoadMatrix, TranslateCase4) {
    polyLine2f line;
    int lineLength = 10;
    for (int i = 0; i <= lineLength; i++) {
        line.points().push_back(vertex2f(i * 0.1, 0));
    }

    RoadMatrix roadMatrix;
    roadMatrix.initialize(0.1, 1, 0.1, 5);
    roadMatrix.aroundLine(line, vertex2f(0, 0), 0);
    EXPECT_EQ(roadMatrix.length(), 10);
    roadMatrix.aroundLine(line, vertex2f(0.55, 0), 0);
    EXPECT_EQ(roadMatrix.length(), 15);
    roadMatrix.aroundLine(line, vertex2f(0.55, 0), 0);
    EXPECT_EQ(roadMatrix.length(), 15);

    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 0).points.at(0).x, -0.55);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 1).points.at(3).x, -0.55);
    EXPECT_FLOAT_EQ(roadMatrix.cell(0, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 0).points.at(0).x, -0.15);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 1).points.at(3).x, -0.15);
    EXPECT_FLOAT_EQ(roadMatrix.cell(4, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 0).points.at(0).x, 0);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 1).points.at(3).x, 0);
    EXPECT_FLOAT_EQ(roadMatrix.cell(5, 1).points.at(3).y, 0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 0).points.at(0).x, 0.9);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 0).points.at(0).y, -0.1);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 1).points.at(3).x, 0.9);
    EXPECT_FLOAT_EQ(roadMatrix.cell(14, 1).points.at(3).y, 0.1);
}

}  // namespace street_environment
