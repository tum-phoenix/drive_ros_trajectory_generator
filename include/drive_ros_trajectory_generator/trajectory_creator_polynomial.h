
#ifndef SRC_TRAJCTOR_CREATOR_POLYNOMIAL_H
#define SRC_TRAJCTOR_CREATOR_POLYNOMIAL_H
#include <ros/ros.h>
#include <drive_ros_trajectory_generator/trajectory_creator_base.h>

class TrajectoryCreatorPolynomial : public TrajctorCreatorBase{
public:
  TrajectoryCreatorPolynomial(ros::NodeHandle nh, ros::NodeHandle pnh)
  ~TrajectoryCreatorPolynomial();
private:
  void drivingLineCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg);


};

#endif
