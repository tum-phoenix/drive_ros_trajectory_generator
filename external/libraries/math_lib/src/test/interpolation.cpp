#include "gtest/gtest.h"
#include "lms/math/interpolation.h"

TEST(Interpolation, linearInt) {
    using lms::math::linearInterpolation;

    int y;

    // simple interpolation
    y = linearInterpolation<int>(10, 10, 100, 20, 55);
    EXPECT_EQ(15, y);

    // dx == 0
    y = linearInterpolation<int>(10, 10, 10, 100, 10);
    EXPECT_EQ(10, y);

    // extrapolation
    y = linearInterpolation<int>(10, 10, 20, 100, 0);
    EXPECT_EQ(-80, y);
}

TEST(Interpolation, linearFloat) {
    using lms::math::linearInterpolation;

    float y;

    // simple interpolation
    y = linearInterpolation<float>(-10, 1, 10, 2, 0);
    EXPECT_FLOAT_EQ(1.5, y);

    // dx == 0
    y = linearInterpolation<float>(10, 10, 10, 100, 20);
    EXPECT_EQ(10, y);

    // extrapolation
    y = linearInterpolation<float>(1, 0, 2, 0.5, 3);
    EXPECT_FLOAT_EQ(1, y);
}
