#include "drive_ros_trajectory_generator/trajectory_line_creator.h"
#include <drive_ros_uavcan/phoenix_msgs__NucDriveCommand.h>
#include <drive_ros_>
#include <drive_ros_trajectory_generator/polygon_msg_operations.h>
#ifdef SUBSCRIBE_DEBUG
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#endif

namespace trajectory_generator {

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

TrajectoryLineCreator::TrajectoryLineCreator(ros::NodeHandle nh, ros::NodeHandle pnh)
	: pnh_(pnh)
	, reconfigure_server_()
#ifdef SUBSCRIBE_DEBUG
  , it_(nh)
  , image_operator_()
#endif
{
  reconfigure_server_.setCallback(boost::bind(&TrajectoryLineCreator::reconfigureCB, this, _1, _2));
  trajectory_meta_sub_ = nh_.subscribe("meta_in", 10, &TrajectoryLineCreator::metaInputCB, this);
  signs_input = nh_.subscribe("signs", 10, &TrajectoryLineCreator::signInputCB, this);
}

bool TrajectoryLineCreator::init() {
  drivingLineSub = nh_.subscribe("line_in", 2, &TrajectoryLineCreator::drivingLineCB, this);
  ROS_INFO_NAMED(stream_name_, "[Trajectory Generator] Subscribing on topic '%s'", drivingLineSub.getTopic().c_str());

  signSub = nh_.subscribe("line_in", 2, &TrajectoryLineCreator::drivingLineCB, this);

  canPub = nh_.advertise<drive_ros_uavcan::phoenix_msgs__NucDriveCommand>("can_topic", 5);
  ROS_INFO_NAMED(stream_name_, "[Trajectory Generator] Publish uav_can messages on topic '%s'",
                 canPub.getTopic().c_str());

#ifdef SUBSCRIBE_DEBUG
  // common image operations, in this case transformations
  if(!image_operator_.init()) {
      ROS_WARN_STREAM_NAMED(stream_name_, "[Trajectory Generator] Failed to init image_operator");
      return false;
  }
  debug_image_sub_ = it_.subscribe("debug_image_in", 1, &TrajectoryLineCreator::debugImgCallback, this);
  debug_image_pub_ = it_.advertise("debug_image_out", 1);
#endif

    return true;
}

#ifdef SUBSCRIBE_DEBUG
// todo: check if this does not sync to the line information properly, in that case conditionally compile message filter
void TrajectoryLineCreator::debugImgCallback(const sensor_msgs::ImageConstPtr& img_in)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_in, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  debug_img_ = cv_ptr->image;
}
#endif

void TrajectoryLineCreator::metaInputCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg) {
  vMax = msg->max_speed;
  drivingCommand = msg->control_metadata;
}

void TrajectoryLineCreator::signCB(const drive_ros_msgs::TrafficMarkEnvironmentConstPtr &msg) {
  int signId=msg->id;
	switch
	vSign =
  drivingCommand = msg->control_metadata;
}

void TrajectoryLineCreator::drivingLineCB(const drive_ros_msgs::DrivingLineConstPtr &msg) {
	// ===========================
	// 			velocity
	// ===========================
  ROS_INFO_NAMED(stream_name_, "---");
  ROS_INFO_NAMED(stream_name_, "detection range = %.2f", msg->detectionRange);

	// calculate forward velocity
	float forwardDistanceX = minForwardDist + std::abs(currentVelocity) * k1;
  forwardDistanceX = hardcodedForwardDistance; //std::min(forwardDistanceX, msg->detectionRange); // limit to detectionRange

	// get y from polynom
	float forwardDistanceY = 0.f;

  // handle lane changes and hard-coded turn/drive commands
  float laneChangeDistance = 0.f;
  float presetSteeringAngle;
  bool steeringAngleFixed = false;
  bool steerFrontAndRear = false;

  drive_ros_uavcan::phoenix_msgs__NucDriveCommand::_blink_com_type blink_com =
          drive_ros_uavcan::phoenix_msgs__NucDriveCommand::NO_BLINK;
  switch (drivingCommand) {
    case (drive_ros_msgs::TrajectoryMetaInput::STANDARD):
      // nothing to do
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_LEFT):
      // shift lane distance to the left
      laneChangeDistance = laneWidth;
      blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_LEFT;
      steerFrontAndRear = true;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_RIGHT):
      // shift lane distance to the right
      laneChangeDistance = -laneWidth;
      blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_RIGHT;
      steerFrontAndRear = true;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::TURN_LEFT):
      // hard-code steering angle to the left
      presetSteeringAngle = crossingTurnAngleLeft;
      steeringAngleFixed = true;
      blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_LEFT;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::TURN_RIGHT):
      // hard code steering angle to the right
      presetSteeringAngle = -crossingTurnAngleRight;
      steeringAngleFixed = true;
      blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_RIGHT;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::STRAIGHT_FORWARD):
      // fix steering to go straight
      presetSteeringAngle = 0.f;
      steeringAngleFixed = true;
    break;
  }
  float Xpoints[20];
  float Ypoints[20];

  forwardDistanceY = compute_polynomial_at_location(msg, forwardDistanceX);

  // compute derivative on carrot point to get normal if we need to offset ortogonally (lane change)
  if (laneChangeDistance != 0.f) {
    float derivative = derive_polynomial_at_location(msg, forwardDistanceX);

    // compute normal vector
    float vec_x = -derivative;
    float vec_y = 1.f;
    float normed_distance = std::sqrt(std::pow(derivative, 2) + std::pow(1.f, 2));
    forwardDistanceX = forwardDistanceX+vec_x*(laneChangeDistance/normed_distance);
    forwardDistanceY = forwardDistanceY+vec_y*(laneChangeDistance/normed_distance);
  }

  float kappa;
  if (!steeringAngleFixed)
    kappa = (std::atan2(forwardDistanceY, forwardDistanceX));
  else
    kappa = presetSteeringAngle;

  ROS_INFO_NAMED(stream_name_, "Goal point (%.2f, %.2f)", forwardDistanceX, forwardDistanceY);
  ROS_INFO_NAMED(stream_name_, "Kappa = %.5f", kappa);

	// TODO: querbeschleunigung

  float vGoal = vMax - std::abs(kappa) * (vMax - vMin);

  ROS_INFO_NAMED(stream_name_, "vGoal = %f", vGoal);

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

  ROS_INFO_NAMED(stream_name_, "polynom(x)  = %.2f", forwardDistanceY);
  ROS_INFO_NAMED(stream_name_, "polynom'(x) = %.2f", phiAtGoalX);

	// radius is also turnRadiusY
	float radius = forwardDistanceY / (1.f - std::sin(M_PI_2 - phiAtGoalX));

  float turnRadiusX = -((forwardDistanceY * std::cos(M_PI_2 - phiAtGoalX)) / (1.f - std::sin(M_PI_2 - phiAtGoalX))) + forwardDistanceX;

  ROS_INFO_NAMED(stream_name_, "Turning point (%.2f, %.2f)", turnRadiusX, radius);

	float steeringAngleRear  = - std::atan(turnRadiusX                  / (radius + (0.001f*(radius == 0.f))));
	float steeringAngleFront = - std::atan((turnRadiusX - axisDistance) / (radius + (0.001f*(radius == 0.f))));

  //ROS_INFO("Steering front = %.1f[deg]", steeringAngleFront * 180.f / M_PI);
  //ROS_INFO("Steering rear  = %.1f[deg]", steeringAngleRear * 180.f / M_PI);

  drive_ros_uavcan::phoenix_msgs__NucDriveCommand driveCmdMsg;
  driveCmdMsg.phi_f = -kappa*understeerFactor;
  if (!steerFrontAndRear)
    driveCmdMsg.phi_r = 0.0f;
  else
    driveCmdMsg.phi_r = -kappa*understeerFactor;
  driveCmdMsg.lin_vel = vGoal;
  driveCmdMsg.blink_com = blink_com;

  //if(!isnanf(steeringAngleFront) && !isnanf(steeringAngleRear)) {
  ROS_INFO_NAMED(stream_name_, "Steering front = %.1f[deg]", driveCmdMsg.phi_f * 180.f / M_PI);
  ROS_INFO_NAMED(stream_name_, "Steering rear  = %.1f[deg]", driveCmdMsg.phi_r * 180.f / M_PI);

  canPub.publish(driveCmdMsg);
  ROS_INFO_NAMED(stream_name_, "Published uavcan message");

#ifdef SUBSCRIBE_DEBUG
  if (!debug_img_.empty()) {
      ROS_INFO_STREAM("Drawing debug image");
      // draw carrot steering point
      std::vector<cv::Point2f> world_point_vec{cv::Point2f(forwardDistanceX, forwardDistanceY)};
      std::vector<cv::Point2f> image_point_vec;
      image_operator_.worldToWarpedImg(world_point_vec, image_point_vec);
      cv::drawMarker(debug_img_, image_point_vec[0], cv::Scalar(0, 0, 255), cv::MARKER_CROSS);
      // draw steering angle
      // baseline point around the center of the front axis, and angle point in front of it
      world_point_vec.clear();
      image_point_vec.clear();
      float forward_draw_distance = 0.1f;
      world_point_vec.push_back(cv::Point2f(0.1f, 0.f));
      world_point_vec.push_back(cv::Point2f(0.1f+forward_draw_distance,
                                            std::tan(kappa*understeerFactor)*forward_draw_distance));
      image_operator_.worldToWarpedImg(world_point_vec, image_point_vec);
      cv::arrowedLine(debug_img_, image_point_vec[0], image_point_vec[1], cv::Scalar(0, 0, 255), 2);
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_img_).toImageMsg();
      debug_image_pub_.publish(msg);
  }
#endif
  //}
}

void TrajectoryLineCreator::reconfigureCB(trajectory_generator::TrajectoryLineCreationConfig& config, uint32_t level) {
  minForwardDist = config.min_forward_dist;
  currentVelocity = config.current_velocity;
  crossingTurnAngleLeft = config.crossing_turn_angle_left;
  crossingTurnAngleRight = config.crossing_turn_angle_right;
  laneWidth = config.lane_width;
  hardcodedForwardDistance = config.hardcoded_forward_distance;
  understeerFactor = config.understeer_factor;
}

} // end namespace trajectory_generator
