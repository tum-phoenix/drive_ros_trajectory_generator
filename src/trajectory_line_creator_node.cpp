#include <ros/ros.h>
#include <drive_ros_trajectory_generator/trajectory_line_creator.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_line_creator_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    trajector_generator::TrajectoryLineCreator line_creator(nh, pnh);
    if (!line_creator.init()) {
        return 1;
    } else {
        ROS_INFO("Trajectory Line Creator node succesfully initialized");
    }

#ifndef NDEBUG
    // give GDB time to attach
    ros::Duration(1.0).sleep();
#endif

    while (ros::ok()) {
        ros::spin();
    }
    return 0;
}

