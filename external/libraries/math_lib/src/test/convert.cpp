#include "gtest/gtest.h"
#include "lms/math/convert.h"

TEST(Convert, deg2rad) {
    using lms::math::Convert;
    
    double result;
    
    // Test conversion
    result = Convert::deg2rad(45.);
    ASSERT_DOUBLE_EQ(M_PI * 0.25, result);
    
    // Zero
    result = Convert::deg2rad(0.);
    ASSERT_DOUBLE_EQ(0., result);
    
    // 360deg
    result = Convert::deg2rad(360.);
    ASSERT_DOUBLE_EQ(M_PI * 2., result);
    
    // Conversion for negative
    result = Convert::deg2rad(-45.);
    ASSERT_DOUBLE_EQ(-M_PI * 0.25, result);
    
    // Conversion for > 360deg
    result = Convert::deg2rad(405.);
    ASSERT_DOUBLE_EQ(M_PI * 2.25, result);
}

TEST(Convert, rad2deg) {
    using lms::math::Convert;
    
    double result;
    
    // Test conversion
    result = Convert::rad2deg(M_PI * 0.75);
    ASSERT_DOUBLE_EQ(135., result);
    
    // Zero
    result = Convert::rad2deg(0.);
    ASSERT_DOUBLE_EQ(0., result);
    
    // 360deg
    result = Convert::rad2deg(M_PI * 2.);
    ASSERT_DOUBLE_EQ(360., result);
    
    // Conversion for negative
    result = Convert::rad2deg(-M_PI * 0.75);
    ASSERT_DOUBLE_EQ(-135., result);
    
    // Conversion for > 360deg
    result = Convert::rad2deg(M_PI * 2.75);
    ASSERT_DOUBLE_EQ(495., result);
}

TEST(Convert, in2m) {
    using lms::math::Convert;
    
    double result;
    
    // Test conversion
    result = Convert::in2m(1.0);
    ASSERT_DOUBLE_EQ(0.0254, result);
    
    // Conversion for negative
    result = Convert::in2m(-2.0);
    ASSERT_DOUBLE_EQ(-0.0508, result);
}

TEST(Convert, m2in) {
    using lms::math::Convert;
    
    double result;
    
    // Test conversion
    result = Convert::m2in(0.0254);
    ASSERT_DOUBLE_EQ(1.0, result);
    
    // Conversion for negative
    result = Convert::m2in(-0.0508);
    ASSERT_DOUBLE_EQ(-2.0, result);
}

TEST(Convert, fromPrefix) {
    using lms::math::Convert;
    
    // Test conversion
    float result = Convert::fromPrefix<std::milli, float>(1234);
    ASSERT_FLOAT_EQ(1.234f, result);
}

TEST(Convert, toPrefix) {
    using lms::math::Convert;
    
    // Test conversion
    uint32_t result = Convert::toPrefix<std::nano, uint32_t>(1.234567890);
    ASSERT_EQ(uint32_t(1234567890), result);
}



