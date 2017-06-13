#include <vector>

#include "lms/math/lookup_table.h"
#include "gtest/gtest.h"

TEST(LookupTable, binarySearchFloat) {
    using lms::math::lookupTableBinarySearch;
    using lms::math::LookupTableOrder;

    const std::vector<float> vx = {705, 507, 348, 270, 209, 178, 142, 115, 106,
                                   97,  83,  70,  65,  65,  60,  50,  42};
    const std::vector<float> vy = {0,  50, 100, 150, 200, 250, 300, 350, 400,
                                   450, 500, 550, 600, 650, 700, 750, 800};
    float y, yExpect;
    bool result;

    result = lookupTableBinarySearch<float, LookupTableOrder::DESC>(vx, vy, 270, y);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(150, y);

    result = lookupTableBinarySearch<float, LookupTableOrder::DESC>(vx, vy, 240, y);
    EXPECT_TRUE(result);
    yExpect = lms::math::linearInterpolation<float>(270, 150, 209, 200, 240);
    EXPECT_FLOAT_EQ(yExpect, y);

    // out of range
    result = lookupTableBinarySearch<float, LookupTableOrder::DESC>(vx, vy, 706, y);
    EXPECT_FALSE(result);

    result = lookupTableBinarySearch<float, LookupTableOrder::DESC>(vx, vy, 65, y);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(600, y);
}

TEST(LookupTable, binarySearchInt) {
    using lms::math::lookupTableBinarySearch;
    using lms::math::LookupTableOrder;

    const std::vector<int> vx = {705, 507, 348, 270, 209, 178, 142, 115, 106,
                                   97,  83,  70,  65,  65,  60,  50,  42};
    const std::vector<int> vy = {0,  50, 100, 150, 200, 250, 300, 350, 400,
                                   450, 500, 550, 600, 650, 700, 750, 800};
    int y;
    bool result;

    result = lookupTableBinarySearch<int, LookupTableOrder::DESC>(vx, vy, 270, y);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(150, y);

    result = lookupTableBinarySearch<int, LookupTableOrder::DESC>(vx, vy, 55, y);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(725, y);

    // NOTE: 65 is duplicated in vx, expect first corresponding vy value
    result = lookupTableBinarySearch<int, LookupTableOrder::DESC>(vx, vy, 65, y);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(600, y);

    // out of range
    result = lookupTableBinarySearch<int, LookupTableOrder::DESC>(vx, vy, 706, y);
    EXPECT_FALSE(result);

    result = lookupTableBinarySearch<int, LookupTableOrder::DESC>(vx, vy, 0, y);
    EXPECT_FALSE(result);
}
