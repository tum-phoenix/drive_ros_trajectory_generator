#include "gtest/gtest.h"
#include "lms/math/pose.h"


TEST(Pose2DHistory,getPose){
    lms::math::Pose2DHistory pHis;
    pHis.addPose(0,0,0,0);
    pHis.addPose(1,2,3,1);
    lms::math::Pose2D pose;
    ASSERT_EQ(pHis.poses().size(),2);
    ASSERT_EQ(pHis.poses()[0].x,0);
    ASSERT_EQ(pHis.poses()[0].y,0);
    ASSERT_EQ(pHis.poses()[0].phi,0);
    ASSERT_EQ(pHis.poses()[0].timeStamp,0);
    ASSERT_EQ(pHis.poses()[1].x,1);
    ASSERT_EQ(pHis.poses()[1].y,2);
    ASSERT_EQ(pHis.poses()[1].phi,3);
    ASSERT_EQ(pHis.poses()[1].timeStamp,1);

    ASSERT_TRUE(pHis.getPose(0.5,pose));
    EXPECT_FLOAT_EQ(0.5, pose.x);
    EXPECT_FLOAT_EQ(1, pose.y);
    EXPECT_FLOAT_EQ(1.5, pose.phi);
    EXPECT_FLOAT_EQ(0.5, pose.timeStamp);
}

TEST(CoordinateSystem2D,fromPose){
    lms::math::Pose2D pose;
    pose.x = 1;
    pose.y = 1;
    pose.phi = 1;
    lms::math::CoordinateSystem2D coord(pose);
    EXPECT_FLOAT_EQ(pose.x,coord.x);
    EXPECT_FLOAT_EQ(pose.y,coord.y);
    EXPECT_FLOAT_EQ(pose.phi,coord.phi);
}

TEST(CoordinateSystem2D,transformTo){
    lms::math::Pose2D pose,newPose;
    pose.x = 1;
    pose.y = 2;
    pose.phi = 1;
    pose.timeStamp = 1;
    lms::math::CoordinateSystem2D coord;
    newPose = coord.transformTo(pose);
    EXPECT_FLOAT_EQ(pose.x,newPose.x);
    EXPECT_FLOAT_EQ(pose.y,newPose.y);
    EXPECT_FLOAT_EQ(pose.phi,newPose.phi);
    EXPECT_FLOAT_EQ(pose.timeStamp,newPose.timeStamp);

    coord.x = 1;
    newPose = coord.transformTo(pose);
    EXPECT_FLOAT_EQ(pose.x-1,newPose.x);
    EXPECT_FLOAT_EQ(pose.y,newPose.y);
    EXPECT_FLOAT_EQ(pose.phi,newPose.phi);
    EXPECT_FLOAT_EQ(pose.timeStamp,newPose.timeStamp);

    coord.x = 0;
    coord.y = 1;
    newPose = coord.transformTo(pose);
    EXPECT_FLOAT_EQ(pose.x,newPose.x);
    EXPECT_FLOAT_EQ(pose.y-1,newPose.y);
    EXPECT_FLOAT_EQ(pose.phi,newPose.phi);
    EXPECT_FLOAT_EQ(pose.timeStamp,newPose.timeStamp);

    coord.x = 1;
    coord.y = 1;
    newPose = coord.transformTo(pose);
    EXPECT_FLOAT_EQ(pose.x-1,newPose.x);
    EXPECT_FLOAT_EQ(pose.y-1,newPose.y);
    EXPECT_FLOAT_EQ(pose.phi,newPose.phi);
    EXPECT_FLOAT_EQ(pose.timeStamp,newPose.timeStamp);

    pose.phi = 0;
    coord.x = 0;
    coord.y = 0;
    coord.phi = M_PI_2;
    newPose = coord.transformTo(pose);
    EXPECT_FLOAT_EQ(pose.y,newPose.x);
    EXPECT_FLOAT_EQ(-pose.x,newPose.y);
    EXPECT_FLOAT_EQ(-M_PI_2,newPose.phi);
    EXPECT_FLOAT_EQ(pose.timeStamp,newPose.timeStamp);


    pose.phi = 0;
    coord.x = 1;
    coord.y = 1;
    coord.phi = M_PI_2;
    newPose = coord.transformTo(pose);
    EXPECT_NEAR(1,newPose.x,1e-6);
    EXPECT_NEAR(0,newPose.y,1e-6);
    EXPECT_NEAR(-M_PI_2,newPose.phi,1e-6);
    EXPECT_FLOAT_EQ(pose.timeStamp,newPose.timeStamp);
}
