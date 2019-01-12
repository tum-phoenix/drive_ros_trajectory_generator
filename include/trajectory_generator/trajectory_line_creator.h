#ifndef TRAJECTORY_LINE_CREATOR_H
#define TRAJECTORY_LINE_CREATOR_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "drive_ros_msgs/DrivingLine.h"
#include "trajectory_generator/TrajectoryLineCreationConfig.h"

#include <drive_ros_msgs/TrajectoryMetaInput.h>

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
    // hardcoded for debugging purposes
    float hardcodedForwardDistance = 1.f;

    //  for commands from BT
    float laneWidth = 0.4f;
    float crossingTurnAngle = 0.4f;
    short int drivingCommand = drive_ros_msgs::TrajectoryMetaInput::STANDARD;

    void drivingLineCB(const drive_ros_msgs::DrivingLineConstPtr &msg);
    void metaInputCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg);

    // Dynamic reconfigure
    void reconfigureCB(trajectory_generator::TrajectoryLineCreationConfig& config, uint32_t level);
    dynamic_reconfigure::Server<trajectory_generator::TrajectoryLineCreationConfig> reconfigure_server_;
};

} // namespace trajectory_generator

#endif // TRAJECTORY_LINE_CREATOR_H
