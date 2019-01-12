#include "trajectory_generator/trajectory_line_creator.h"
#include <drive_ros_uavcan/phoenix_msgs__NucDriveCommand.h>

namespace trajectory_generator {

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

TrajectoryLineCreator::TrajectoryLineCreator(ros::NodeHandle nh, ros::NodeHandle pnh)
	: pnh_(pnh)
	, reconfigure_server_()
{
	reconfigure_server_.setCallback(boost::bind(&TrajectoryLineCreator::reconfigureCB, this, _1, _2));
  trajectory_meta_sub_ = pnh_.subscribe("meta_in", 10, &TrajectoryLineCreator::metaInputCB, this);
}

bool TrajectoryLineCreator::init() {
	drivingLineSub = nh_.subscribe("line_in", 2, &TrajectoryLineCreator::drivingLineCB, this);
	ROS_INFO("Subscribing on topic '%s'", drivingLineSub.getTopic().c_str());

	canPub = nh_.advertise<drive_ros_uavcan::phoenix_msgs__NucDriveCommand>("can_topic", 5);
	ROS_INFO("Publish uav_can messages on topic '%s'", canPub.getTopic().c_str());

	return true;
}

void TrajectoryLineCreator::metaInputCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg) {
  vMax = msg->max_speed;
  drivingCommand = msg->control_metadata;
}

void TrajectoryLineCreator::drivingLineCB(const drive_ros_msgs::DrivingLineConstPtr &msg) {
	// ===========================
	// 			velocity
	// ===========================
	ROS_INFO("---");
	ROS_INFO("detection range = %.2f", msg->detectionRange);

	// calculate forward velocity
	float forwardDistanceX = minForwardDist + std::abs(currentVelocity) * k1;
  forwardDistanceX = hardcodedForwardDistance; //std::min(forwardDistanceX, msg->detectionRange); // limit to detectionRange

	// get y from polynom
	float forwardDistanceY = 0.f;

  // handle lane changes and hard-coded turn/drive commands
  float laneChangeDistance = 0.f;
  float presetSteeringAngle;
  bool steeringAngleFixed = false;
  switch (drivingCommand) {
    case (drive_ros_msgs::TrajectoryMetaInput::STANDARD):
      // nothing to do
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_LEFT):
      // shift lane distance to the left
      laneChangeDistance = laneWidth;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_RIGHT):
      // shift lane distance to the right
      laneChangeDistance = laneWidth;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::TURN_LEFT):
      // hard-code steering angle to the left
      presetSteeringAngle = steeringAngleFixed;
      steeringAngleFixed = true;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::TURN_RIGHT):
      // hard code steering angle to the right
      presetSteeringAngle = -steeringAngleFixed;
      steeringAngleFixed = true;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::STRAIGHT_FORWARD):
      // fix steering to go straight
      presetSteeringAngle = 0.f;
      steeringAngleFixed = true;
    break;
  }

  for(int i = 0; i <= msg->polynom_order; i++) {
    forwardDistanceY += msg->polynom_params.at(i)*std::pow(forwardDistanceX, i);
  }

  // compute derivative on carrot point to get normal if we need to offset ortogonally (lane change)
  if (laneChangeDistance != 0.f) {
    float derivative = 0.f;
    for(int i = 0; i <= msg->polynom_order-1; i++) {
      derivative += msg->polynom_params.at(i+1)*(i+1)*std::pow(forwardDistanceX, i);
    }

    // compute normal vector
    float vec_x = -derivative;
    float vec_y = 1.f;
    float normed_distance = std::sqrt(std::pow(derivative, 2) + std::pow(1.f, 2));
    forwardDistanceX = forwardDistanceX+vec_x*(laneWidth/normed_distance);
    forwardDistanceY = forwardDistanceY+vec_y*(laneWidth/normed_distance);
  }

  float kappa = (std::atan2(forwardDistanceY, forwardDistanceX));

	ROS_INFO("Goal point (%.2f, %.2f)", forwardDistanceX, forwardDistanceY);
	ROS_INFO("Kappa = %.5f", kappa);

	// TODO: querbeschleunigung

  float vGoal = vMax - std::abs(kappa) * (vMax - vMin);

	ROS_INFO("vGoal = %f", vGoal);

	// ===========================
	// 		steering angles
	// ===========================

	float phiAtGoalX = 0.f; // deviate the polynom
	for(int i = 1; i <= msg->polynom_order; i++) {
		float tmp = msg->polynom_params.at(i) * i;
		for(int j = 1; j < i; j++) {
			tmp *= forwardDistanceX;
		}
		phiAtGoalX += tmp;
	}

	ROS_INFO("polynom(x)  = %.2f", forwardDistanceY);
	ROS_INFO("polynom'(x) = %.2f", phiAtGoalX);

	// radius is also turnRadiusY
	float radius = forwardDistanceY / (1.f - std::sin(M_PI_2 - phiAtGoalX));

  float turnRadiusX = -((forwardDistanceY * std::cos(M_PI_2 - phiAtGoalX)) / (1.f - std::sin(M_PI_2 - phiAtGoalX))) + forwardDistanceX;

	ROS_INFO("Turning point (%.2f, %.2f)", turnRadiusX, radius);

	float steeringAngleRear  = - std::atan(turnRadiusX                  / (radius + (0.001f*(radius == 0.f))));
	float steeringAngleFront = - std::atan((turnRadiusX - axisDistance) / (radius + (0.001f*(radius == 0.f))));

  //ROS_INFO("Steering front = %.1f[deg]", steeringAngleFront * 180.f / M_PI);
  //ROS_INFO("Steering rear  = %.1f[deg]", steeringAngleRear * 180.f / M_PI);

  drive_ros_uavcan::phoenix_msgs__NucDriveCommand driveCmdMsg;
  driveCmdMsg.phi_f = kappa*0.65;
  driveCmdMsg.phi_r = 0.0f;
  driveCmdMsg.lin_vel = vGoal;

  //if(!isnanf(steeringAngleFront) && !isnanf(steeringAngleRear)) {
  ROS_INFO("Steering front = %.1f[deg]", driveCmdMsg.phi_f * 180.f / M_PI);
  ROS_INFO("Steering rear  = %.1f[deg]", driveCmdMsg.phi_r * 180.f / M_PI);

  canPub.publish(driveCmdMsg);
  ROS_INFO("Published uavcan message");
  //}
}

void TrajectoryLineCreator::reconfigureCB(trajectory_generator::TrajectoryLineCreationConfig& config, uint32_t level) {
	minForwardDist = config.min_forward_dist;
	currentVelocity = config.current_velocity;
  crossingTurnAngle = config.crossing_turn_angle;
  laneWidth = config.lane_width;
  hardcodedForwardDistance = config.hardcoded_forward_distance;
}

} // end namespace trajectory_generator
