#include <cmath>

#include "gtest/gtest.h"
#include "lms/math/polyline.h"

namespace lms {
namespace math {

TEST(Polyline2f,firstOrthogonalDistance){
    lms::math::polyLine2f l;
    l.points().push_back(lms::math::vertex2f(0,0));
    l.points().push_back(lms::math::vertex2f(1,0));
    EXPECT_EQ(2,l.points().size());
    float t,o;
    l.firstOrthogonalDistance(lms::math::vertex2f(0.5,0),o,t);
    EXPECT_FLOAT_EQ(0,o);
    EXPECT_FLOAT_EQ(0.5,t);
    l.firstOrthogonalDistance(lms::math::vertex2f(0.5,0.5),o,t);
    EXPECT_FLOAT_EQ(0.5,o);
    EXPECT_FLOAT_EQ(0.5,t);
    l.firstOrthogonalDistance(lms::math::vertex2f(0.5,-0.5),o,t);
    EXPECT_FLOAT_EQ(-0.5,o);
    EXPECT_FLOAT_EQ(0.5,t);
    l.firstOrthogonalDistance(lms::math::vertex2f(-0.5,-0.5),o,t);
    EXPECT_FLOAT_EQ(-0.5,o);
    EXPECT_FLOAT_EQ(-0.5,t);
}
TEST(Polyline2f,firstOrthogonalDistance_2){
    lms::math::polyLine2f l;
    l.points().push_back(lms::math::vertex2f(1,1));
    l.points().push_back(lms::math::vertex2f(2,2));
    EXPECT_EQ(2,l.points().size());
    float t,o;
    l.firstOrthogonalDistance(lms::math::vertex2f(0,0),o,t);
    EXPECT_FLOAT_EQ(0,o);
    EXPECT_FLOAT_EQ(-std::sqrt(2),t);
    l.points().clear();
    l.points().push_back(lms::math::vertex2f(0,0.2));
    l.points().push_back(lms::math::vertex2f(0.3,0.2));
    l.firstOrthogonalDistance(lms::math::vertex2f(0.06,-0.16),o,t);
    EXPECT_FLOAT_EQ(-0.36,o);
    EXPECT_FLOAT_EQ(0.06,t);
    l.firstOrthogonalDistance(lms::math::vertex2f(0.06,0.1),o,t);
    EXPECT_FLOAT_EQ(-0.1,o);
    EXPECT_FLOAT_EQ(0.06,t);


    /*
    l.firstOrthogonalDistance(lms::math::vertex2f(1,0),o,t);
    EXPECT_FLOAT_EQ(-0.5,o);
    EXPECT_FLOAT_EQ(0,t);

    l.firstOrthogonalDistance(lms::math::vertex2f(0.5,-0.5),o,t);
    EXPECT_FLOAT_EQ(-0.5,o);
    EXPECT_FLOAT_EQ(0.5,t);
    l.firstOrthogonalDistance(lms::math::vertex2f(-0.5,-0.5),o,t);
    EXPECT_FLOAT_EQ(-0.5,o);
    EXPECT_FLOAT_EQ(-0.5,t);
    */
}


}
}
