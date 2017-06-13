#include <ros/ros.h>
#include <trajectory_generator/trajectory_line_creator.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_line_creator_node");
  ros::NodeHandle nh("~");

  trajectory_generator::TrajectoryLineCreator line_creator = trajectory_generator::TrajectoryLineCreator(nh);
  if (!line_creator.init()) {
    return 1;
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

