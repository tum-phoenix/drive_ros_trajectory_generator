//
// Created by seb on 06.12.20.
//

#ifndef SRC_TRAJCTOR_CREATOR_BASE_H
#define SRC_TRAJCTOR_CREATOR_BASE_H

#include <ros/ros.h>
#include "drive_ros_msgs/DrivingLine.h"
#include "drive_ros_msgs/TrajectoryMetaInput.h"

//  for commands from BT
// width of a single lane
const float laneWidth = 0.4f;  //0.350-0.450
// steering angle when turning left at an intersection
const float crossingTurnAngleLeft = 0.35f;
//steering angle when turning right at an intersection
const float crossingTurnAngleRight = 0.6f;

class TrajectoryCreatorBase {

public:
    TrajectoryCreatorBase(ros::NodeHandle nh, ros::NodeHandle pnh);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber driving_line_sub_;
    ros::Subscriber trajectory_meta_sub_;
    ros::Publisher trajectoryPub_;

    float current_desired_velocity_;
    float next_desired_velocity_;
    float distance_to_react_;


    short int driving_command = drive_ros_msgs::TrajectoryMetaInput::STANDARD;
    virtual void publishTrajectory()=0;

    void drivingLineCB(const drive_ros_msgs::DrivingLineConstPtr &msg);
    void metaInputCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg);

    drive_ros_msgs::DrivingLine last_driving_line_;

};


#endif //SRC_TRAJCTOR_CREATOR_BASE_H
