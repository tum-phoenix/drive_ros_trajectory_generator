

#include "../include/drive_ros_trajectory_generator/trajectory_creator_polynomial.h"

TrajectoryCreatorPolynomial::TrajectoryCreatorPolynomial(ros::NodeHandle nh, ros::NodeHandle pnh):
        TrajectoryTrackingController(nh, pnh) {

        }

TrajectoryCreatorPolynomial::~TrajectoryCreatorPolynomial() {}

void TrajectoryCreatorPolynomial::drivingLineCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg) {

  

}
