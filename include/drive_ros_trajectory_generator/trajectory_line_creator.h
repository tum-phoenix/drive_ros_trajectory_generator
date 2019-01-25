#ifndef TRAJECTORY_LINE_CREATOR_H
#define TRAJECTORY_LINE_CREATOR_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "drive_ros_msgs/DrivingLine.h"
#include "drive_ros_trajectory_generator/TrajectoryLineCreationConfig.h"

#include <drive_ros_msgs/TrajectoryMetaInput.h>
#ifdef SUBSCRIBE_DEBUG
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#endif
#include <drive_ros_image_recognition/common_image_operations.h>

namespace trajectory_generator{

class TrajectoryLineCreator {

public:
    TrajectoryLineCreator(ros::NodeHandle nh, ros::NodeHandle pnh);
    bool init();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber drivingLineSub;
    ros::Subscriber trajectory_meta_sub_;
    ros::Publisher canPub;

    float currentVelocity; // TODO
    float minForwardDist = 1.0f; // TODO
    float k1 = 1.f / 6.f; // TODO
    float vMax = 2.0f;
    float vMin = 0.1f;
    float axisDistance = 0.222f;
    float understeerFactor = 0.65f;
    // hardcoded for debugging purposes
    float hardcodedForwardDistance = 1.f;

    //  for commands from BT
    float laneWidth = 0.4f;
    float crossingTurnAngle = 0.4f;
    short int drivingCommand = drive_ros_msgs::TrajectoryMetaInput::STANDARD;

    void drivingLineCB(const drive_ros_msgs::DrivingLineConstPtr &msg);
    void metaInputCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg);
#ifdef SUBSCRIBE_DEBUG
    image_transport::ImageTransport it_;
    image_transport::Subscriber debug_image_sub_;
    image_transport::Publisher debug_image_pub_;
    void debugImgCallback(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat debug_img_;
    drive_ros_image_recognition::ImageOperator image_operator_;
#endif

    std::string stream_name_ = "TRAJECTORY_GENERATOR";

    // Dynamic reconfigure
    void reconfigureCB(trajectory_generator::TrajectoryLineCreationConfig& config, uint32_t level);
    dynamic_reconfigure::Server<trajectory_generator::TrajectoryLineCreationConfig> reconfigure_server_;
};

} // namespace trajectory_generator

#endif // TRAJECTORY_LINE_CREATOR_H
