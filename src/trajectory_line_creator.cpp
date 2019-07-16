#include "drive_ros_trajectory_generator/trajectory_line_creator.h"
#include <drive_ros_uavcan/phoenix_msgs__NucDriveCommand.h>
#include <drive_ros_trajectory_generator/polygon_msg_operations.h>
#include <drive_ros_msgs/Trajectory.h>
#include <drive_ros_msgs/TrajectoryPoint.h>
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
  signs_input = nh_.subscribe("signs", 10, &TrajectoryLineCreator::signCB, this);
}

bool TrajectoryLineCreator::init() {
  drivingLineSub = nh_.subscribe("line_in", 2, &TrajectoryLineCreator::drivingLineCB, this);
  ROS_INFO_NAMED(stream_name_, "[Trajectory Generator] Subscribing on topic '%s'", drivingLineSub.getTopic().c_str());

//signSub = nh_.subscribe("line_in", 2, &TrajectoryLineCreator::drivingLineCB, this);

  canPub = nh_.advertise<drive_ros_uavcan::phoenix_msgs__NucDriveCommand>("can_topic", 5);
  Trajectory_publisher = nh_.advertise<drive_ros_msgs::Trajectory>("trajectory_out", 5);
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
  //int signId=msg->id;
	//switch
	//vSign =
  //drivingCommand = msg->control_metadata;
}

void TrajectoryLineCreator::drivingLineCB(const drive_ros_msgs::DrivingLineConstPtr &msg) {
  static int fixed_trajectory=0;
  float laneChangeDistance = 0.f;

  int Num_points=20;
    float Xpoint;
    float Ypoint;
    drive_ros_msgs::Trajectory msg_traj;
    drive_ros_msgs::TrajectoryPoint msg_points;
    float kappa;
    float vGoal;
    for (int count=1;count<Num_points; count++){
        Xpoint=count*0.1;
        Ypoint=compute_polynomial_at_location(msg, Xpoint);
        msg_points.pose.x=Xpoint;
        msg_points.pose.y=Ypoint;
        kappa = (std::atan2(Ypoint, Xpoint));
        vGoal = vMax - std::abs(kappa) * (vMax - vMin);
        msg_points.twist.x=1.0; //vGoal
        msg_traj.points.push_back(msg_points);

    };
  drive_ros_uavcan::phoenix_msgs__NucDriveCommand::_blink_com_type blink_com =
          drive_ros_uavcan::phoenix_msgs__NucDriveCommand::NO_BLINK;
/*
  switch (drivingCommand) {
    case (drive_ros_msgs::TrajectoryMetaInput::STANDARD):
      // nothing to do
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_LEFT):
      // shift lane distance to the left
      fixed_trajectory=2;
      float switch_lane[2]=[0.2,0];
      blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_LEFT;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_RIGHT):
      fixed_trajectory=2;
      blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_RIGHT;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::TURN_LEFT):
      // hard-code steering angle to the left
      fixed_trajectory=8;
      float pointsY=[];


      blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_LEFT;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::TURN_RIGHT):
      // hard code steering angle to the right

      blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_RIGHT;
    break;
    case (drive_ros_msgs::TrajectoryMetaInput::STRAIGHT_FORWARD):
      // fix steering to go straight

    break;
  }
*/
  Trajectory_publisher.publish(msg_traj);
  float forwardDistanceX = minForwardDist + std::abs(currentVelocity) * k1;
  float forwardDistanceY = compute_polynomial_at_location(msg, forwardDistanceX);

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

  float presetSteeringAngle = 0.f;
  bool steeringAngleFixed=false;
  if (!steeringAngleFixed)
    kappa = (std::atan2(forwardDistanceY, forwardDistanceX));
  else
    kappa = presetSteeringAngle;

  ROS_INFO_NAMED(stream_name_, "Goal point (%.2f, %.2f)", forwardDistanceX, forwardDistanceY);
  ROS_INFO_NAMED(stream_name_, "Kappa = %.5f", kappa);

	// TODO: querbeschleunigung



  ROS_INFO_NAMED(stream_name_, "vGoal = %f", vGoal);


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
