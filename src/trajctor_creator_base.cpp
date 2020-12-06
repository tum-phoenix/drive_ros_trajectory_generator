//
// Created by seb on 06.12.20.
//

#include "../include/drive_ros_trajectory_generator/trajctor_creator_base.h"


TrajectorCreatorBase::TrajectorCreatorBase(ros::NodeHandle nh, ros::NodeHandle pnh) : pnh_(pnh){
    trajectory_meta_sub_ = nh_.subscribe("meta_in", 10, &TrajectorCreatorBase::metaInputCB, this);
    drivingLineSub_ = nh_.subscribe("/line_in",10,&TrajectorCreatorBase::drivingLineCB, this);
}

void TrajctorCreatorBase::metaInputCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg) {
    next_desired_velocity_ = msg->max_speed;
    distance_to_react_=msg->distance_to_react;
    driving_command = msg->control_metadata;
}

void TrajectoryCreatorBase::drivingLineCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg){
    //  publish here
}