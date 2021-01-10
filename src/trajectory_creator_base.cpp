//
// Created by seb on 06.12.20.
//

#include <drive_ros_trajectory_generator/trajectory_creator_base.h>


TrajectoryCreatorBase::TrajectoryCreatorBase(ros::NodeHandle nh, ros::NodeHandle pnh) : pnh_(pnh){
    trajectory_meta_sub_ = nh_.subscribe("meta_in", 10, &TrajectoryCreatorBase::metaInputCB, this);
    driving_line_sub_ = nh_.subscribe("/line_in",10,&TrajectoryCreatorBase::drivingLineCB, this);
}

void TrajectoryCreatorBase::metaInputCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg) {
    next_desired_velocity_ = msg->max_speed;
    distance_to_react_=msg->dist_to_react;
    driving_command = msg->control_metadata;
}

void TrajectoryCreatorBase::drivingLineCB(const drive_ros_msgs::DrivingLineConstPtr &msg){
    last_driving_line_ = *msg;
}
