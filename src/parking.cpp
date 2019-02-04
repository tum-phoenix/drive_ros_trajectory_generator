#include <drive_ros_trajectory_generator/parking.h>
#include <limits>
#include <tf/transform_datatypes.h>
#include <drive_ros_environment_model/polygon_msg_operations.h>

Parking::Parking(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : tfBuffer(ros::Duration(10))
, tfListener(tfBuffer)
{
  pnh_ = pnh;
  nh_ = nh;
  car_yawAngle = 0.0;
  car_xPosition = 0.0;
  yawAngleStartEntering = 0.0;
  endX = 0.0;
  y0_dynamic = 0.0;
  ind_end = 0.0;
  straightMove = false;
  correctingCounter = 0;
  yawAngleSet = false;
  finishCounter = 0;
  m_cycleCounter = 0;

  pulloutstate = PullOutState::PULLBACK;
  currentState = ParkingState::SEARCHING;
  firstCircleArc = true;
  parkingSpaceSize = 0.0;

  scan_sub_ = nh_.subscribe("/scan", 2, &Parking::scanCB, this);
  // NOTE: queue was 5 before, changed it down to 1
  drive_command_pub_ = nh_.advertise<drive_ros_uavcan::phoenix_msgs__NucDriveCommand>("can_topic", 1);
  drivingLineSub_ = nh_.subscribe("/driving_line", 2, &Parking::drivingLineCB, this);
  parallelParkingSub_ = nh_.subscribe("parallel_parking", 2, &Parking::parallelParkingCB, this);
  in_progress_service_ = nh_.advertiseService("parking_in_progress", &Parking::parkingInProgressCB, this);

  pnh_.param<float>("wheel_base", lr, 0.23f); //Radstand
  pnh_.param<float>("car_length", l, 0.39f);  //Fahrzeuglänge
  pnh_.param<float>("car_width", b, 0.21f); //Fahrzeugbreite

  currentXPosition = 0;
  lastTimeStamp = -1;
  lastImuTimeStamp = -1;

  move_straight_start_pos = 0;
  finished_pos = 0;
  oldDeltaPhi = 0;
  distanceToObstacleFront = 0;
}

void Parking::scanCB(const sensor_msgs::LaserScanConstPtr &scan){
  ROS_INFO("Scan CB");
    bool validDistanceToObstacleFront = false;
    const float maxDetectionAngle = cfg_.obstacle_detection_angle*M_PI/180;
    if (scan->ranges.size() > 0) {
        float smallestDistance = std::numeric_limits<float>::infinity();
        int added = 0;

        for (int i=0; i<scan->ranges.size(); ++i)
        {
            if(std::fabs(scan->angle_min+scan->angle_increment*i) < maxDetectionAngle) {
                // get smallest distance
                if(scan->ranges[i] < smallestDistance) {
                    smallestDistance = scan->ranges[i];
                    added++;
                }
            }

            // DEBUG TO FIND THE RIGHT ANGLE PERPENDICULAR TO THE CAR
//            if(fabs((scan->angle_min + scan->angle_increment*i) - M_PI_2) < (180.f / M_PI)) {
//            	ROS_INFO("depth at angle %.3f[rad] = %.2f", (scan->angle_min + scan->angle_increment*i), scan->ranges.at(i));
//            }
//            if(fabs((scan->angle_min + scan->angle_increment*i) + M_PI_2) < (180.f / M_PI)) {
//            	ROS_INFO("depth at angle %.3f[rad] = %.2f", (scan->angle_min + scan->angle_increment*i), scan->ranges.at(i));
//            }

//            if(currentState == ParkingState::SEARCHING) {
//              if(((scan->angle_min + scan->angle_increment*i) + M_PI_2) < (1.0 / 180.f * M_PI)) {
//                ROS_INFO("Distance to right is %.4f", scan->ranges.at(i));
//            		if(scan->ranges.at(i) > cfg_.min_gap_depth) {
//                  // start of a gap
//            			if(!measuringGap) {
//            				gapFoundAt = ros::Time::now();
//            			}
//            		} else {
//            			// end of a gap
//            			if(measuringGap) {
//            				measuringGap = false;
//            				geometry_msgs::TransformStamped transform =
//            						tfBuffer.lookupTransform(movingFrame, ros::Time::now(), movingFrame, gapFoundAt, staticFrame);
//            				if(transform.transform.translation.x > cfg_.min_parking_space_size) {
//            					ROS_INFO("--- gap size: %.2f", transform.transform.translation.x);
//            					spaceFound = true;
//            					startParkingStamp = ros::Time::now();
//            				}
//            			}
//            		}
//            	}
//            }
        }

        // check to left side if we find a gap

        if(added!= 0) {
            validDistanceToObstacleFront = true;
            distanceToObstacleFront = smallestDistance;
            ROS_INFO_STREAM_NAMED(stream_name_, "[PARKING] Distance to the obstacle up front "<<distanceToObstacleFront);
        }
    }else{
        ROS_WARN_STREAM_NAMED(stream_name_, "[PARKING] No lidar data given!");
    }
}

void Parking::drivingLineCB(const drive_ros_msgs::DrivingLineConstPtr &drivingLineMsg) {
  drivingLineReceived = true;
  currentDrivingLine = drivingLineMsg;
}

void Parking::parallelParkingCB(const drive_ros_uavcan::phoenix_msgs__ParallelParkingConstPtr &parallelParkingMsg)
{
  ROS_ERROR("PARKING GAP RECEIVED!");
  spaceFound = true;
  startParkingStamp = ros::Time::now();
}

bool Parking::parkingInProgressCB(drive_ros_msgs::ParkingInProgress::Request &req,
                                  drive_ros_msgs::ParkingInProgress::Response &res)
{
  res.parking_in_progress = parkingInProgress;
  return true;
}

void Parking::setReconfigure(const drive_ros_trajectory_generator::TrajectoryLineCreationConfig cfg)
{
    cfg_ = cfg;
}

bool Parking::triggerParking() {
  ROS_INFO_NAMED(stream_name_, "[PARKING] Starting to park");
  if(parkingInProgress)
    return false;

//  if(m_cycleCounter > 10){
//    ROS_ERROR_STREAM("yaw angle: "<< car_yawAngle*(float)180/M_PI);

//    if(cfg_.yaw_threshold >= std::fabs())
//    {
//      car_yawAngle += -car->deltaPhi();
//      oldDeltaPhi = car->deltaPhi();
//    } else {
//      car_yawAngle += -oldDeltaPhi;
//    }
//  }
  done = false;
  drive_ros_uavcan::phoenix_msgs__NucDriveCommand drive_command;
  switch (currentState)
  {
  case ParkingState::SEARCHING:
  {
    ROS_INFO("Parking: SEARCHING");

    // turn of indicators
    drive_command.blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::NO_BLINK;

    /***************************************************
         * drive straight along the middle of the right lane
         ***************************************************/


    // the desired state is such that phi=0 (straight driving) and y=0.2 (middle of right lane) with respect to the middle lane
//    setSteeringAngles(drive_command, cfg_.obstacle_detection_angle, cfg_.searching_angle, DrivingMode::FORWARD);
    drive_command.phi_f = 0.0;
    drive_command.phi_r = 0.0;

    drive_command.lin_vel = cfg_.velocity_searching;

    // check if we detected a valid parking space
//    bool spaceFound = checkForGap();
    ROS_DEBUG_STREAM_NAMED(stream_name_, "[PARKING] Searching for gap, found "<<spaceFound);

    if (spaceFound) {
      currentState = ParkingState::STOPPING;
      startParkingStamp = ros::Time::now();
    }
//    parkingInProgress = false;

    break;
  }
    //Anhalten nach Searching
  case ParkingState::STOPPING:
  {
    ROS_INFO_NAMED(stream_name_, "STOPPING");

    /***************************************************
        * come to a stop, but dont slip (otherwise the position measurements are crap)
        ***************************************************/

    //drive straight still
//    setSteeringAngles(drive_command, cfg_.searching_middle_offset, cfg_.searching_phi_factor, DrivingMode::FORWARD);
    drive_command.phi_f = 0.0;
    drive_command.phi_r = 0.0;
    drive_command.lin_vel = 0.0;

    drive_command_pub_.publish(drive_command);
    currentState = ParkingState::ENTERING;
    break;


    //update current x-position
//    updatePositionFromHall();

    //set target speed as if the car was decelerating constantly
//    if (cfg_.deceleration_stopping) {
//      drive_command.lin_vel = cfg_.velocty_searching - (ros::Time::now()-startParkingStamp).toSec()*
//              cfg_.deceleration_stopping;
//    }
//    else
//    {
//      drive_command.lin_vel = 0.0;
//    }

//    //int num_y_vals = config().get("numberOfY0measurements", 20);
//    // todo: get velocity from TF here
//    double dur = 0.1;
//    geometry_msgs::TransformStamped transform =
//    		tfBuffer.lookupTransform(movingFrame, ros::Time::now(), movingFrame, ros::Time::now() - ros::Duration(dur), staticFrame);

//    auto xTrans = transform.transform.translation.x;
//    auto yTrans = transform.transform.translation.y;

//    auto velocity = sqrt(xTrans*xTrans + yTrans*yTrans) / dur; // [m/s]

//    if (velocity < cfg_.min_velocity_before_driving_backwards) {

//      y0_dynamic = cfg_.y0_worstCase;

//      currentState = ParkingState::ENTERING;
//    }

//    break;
//    parkingInProgress =false;
  }

  case ParkingState::ENTERING:
  {
    parkingInProgress = true;
    ros::Time start_time = ros::Time::now();

    ROS_ERROR("STRAIGHT");
    while (ros::Time::now().toSec() < (start_time+ros::Duration(0.4)).toSec()) {
      drive_command.lin_vel = 0.3;
      drive_command.blink_com = drive_command.BLINK_RIGHT;
      drive_command.phi_f = 0.0;
      drive_command.phi_r = 0.0;
      drive_command_pub_.publish(drive_command);
      ros::Duration(0.01).sleep();
    }

    start_time = ros::Time::now();
    while ((ros::Time::now().toSec()) < (start_time+ros::Duration(1.)).toSec()) {
      drive_command.lin_vel = -0.3;
      drive_command.blink_com = drive_command.BLINK_RIGHT;
      drive_command.phi_f = 32*M_PI/180.0;
      drive_command.phi_r = 0.0;
      drive_command_pub_.publish(drive_command);
      ros::Duration(0.01).sleep();
    }

    ROS_ERROR("STRAIGHT");

    start_time = ros::Time::now();
    while (ros::Time::now().toSec() < (start_time+ros::Duration(0.5)).toSec()) {
      drive_command.lin_vel = -0.3;
      drive_command.blink_com = drive_command.BLINK_RIGHT;
      drive_command.phi_f = 0.0;
      drive_command.phi_r = 0.0;
      drive_command_pub_.publish(drive_command);
      ros::Duration(0.01).sleep();
    }

    ROS_ERROR("LEFT");

    start_time = ros::Time::now();
    while (ros::Time::now().toSec() < (start_time+ros::Duration(1.2)).toSec()) {
      drive_command.lin_vel = -0.3;
      drive_command.blink_com = drive_command.BLINK_RIGHT;
      drive_command.phi_f = -32*M_PI/180.0;
      drive_command.phi_r = 15*M_PI/180.0;
      drive_command_pub_.publish(drive_command);
      ros::Duration(0.01).sleep();
    }

  ROS_ERROR("WAITING");

    start_time = ros::Time::now();
    while (ros::Time::now().toSec() < (start_time+ros::Duration(3.0)).toSec()) {
      drive_command.lin_vel = 0.0;
      drive_command.blink_com = drive_command.BLINK_BOTH;
      drive_command_pub_.publish(drive_command);
      ros::Duration(0.01).sleep();
    }

    ROS_ERROR("OUT LEFT");

    // drive out again
    start_time = ros::Time::now();
    while (ros::Time::now().toSec() < (start_time+ros::Duration(1.)).toSec()) {
      drive_command.lin_vel = 0.3;
      drive_command.blink_com = drive_command.BLINK_LEFT;
      drive_command.phi_f = -32*M_PI/180.0;
      drive_command.phi_r = 0.0;
      drive_command_pub_.publish(drive_command);
      ros::Duration(0.01).sleep();
    }

    ROS_ERROR("OUT STRAIGHT");

    start_time = ros::Time::now();
    while (ros::Time::now().toSec() < (start_time+ros::Duration(0.7)).toSec()) {
      drive_command.lin_vel = 0.3;
      drive_command.blink_com = drive_command.BLINK_LEFT;
      drive_command.phi_f = 0.0;
      drive_command.phi_r = 0.0;
      drive_command_pub_.publish(drive_command);
      ros::Duration(0.01).sleep();
    }

    ROS_ERROR("OUT RIGHT");

    start_time = ros::Time::now();
    while (ros::Time::now().toSec() < (start_time+ros::Duration(1.)).toSec()) {
      drive_command.lin_vel = 0.3;
      drive_command.blink_com = drive_command.BLINK_LEFT;
      drive_command.phi_f = 32*M_PI/180.0;
      drive_command.phi_r = -15*M_PI/180.0;
      drive_command_pub_.publish(drive_command);
      ros::Duration(0.01).sleep();
    }

    ROS_ERROR("DONE");

    done = true;



    break;
  }
#if 0
    //Wir fahren in die Parklücke
  case ParkingState::ENTERING:
  {
    ROS_INFO_NAMED(stream_name_, "ENTERING");

    drive_command.blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_RIGHT;

    updatePositionFromHall();

    /***************************************************
        * calculate parameters for entering maneuver
        ***************************************************/
    const double k = cfg_.k; //Sicherheitsabstand zur Ecke der 2. Box
    const double d = cfg_.d; //Sicherheitsabstand zur 1. Box im eingeparkten Zustand
    const double y0 = cfg_.y0_worstCase; //seitlicher Abstand zur zweiten Box

    //TODO delta_max + 1*pi/180;
    float delta_max = cfg_.max_steering_angle * M_PI / 180.f;
    const double r = lr/2*tan(M_PI/2 - (delta_max-1*M_PI/180)); //Radius des Wendekreises (bezogen auf den Fahrzeugmittelpunkt) bei Volleinschlag beider Achsen
    const double R = sqrt(l*l/4 + (r+b/2)*(r+b/2)); //Radius den das äußerste Eck des Fahrzeugs bei volleingeschlagenen Rädern zurücklegt
    const double s = sqrt((R+k)*(R+k) - ( - d - l/2)*( - d - l/2));
    const double alpha = acos((r-y0+s)/(2*r)); //Winkel (in rad) der 2 Kreisboegen, die zum einfahren genutzt werden
    const double x0 = d + l/2 + 2*r*sin(alpha); //Abstand vom Ende der 2. Box zur Mitte des Fahrzeugs bei Lenkbeginn (Anfang erster Kreisbogen)
    const double x_begin_steering = cfg_.x_distance_correction + endX + x0 - cfg_.distance_mid_lidar_x;
    //drive straight backwards
    if (currentXPosition > x_begin_steering){
      ROS_INFO_NAMED(stream_name_, "Driving straight backwards");
      //setSteeringAngles(-0.2, config().get<float>("searchingPhiFactor"), DrivingMode::BACKWARDS);
      drive_command.phi_r = 0.0;
      drive_command.phi_f = 0.0;
      /*
            double brakingDistance = config().get<float>("brakingDistanceUntilSteering", 0.15);
            //TODO evtl
            if (false && currentXPosition < x_begin_steering + brakingDistance)
            {
                double deltaX = currentXPosition - x_begin_steering; //distance until steering
                double deltaV = config().get<float>("velocityApproaching", 1.0) - config().get<float>("velocityEntering", 0.5);
                state.targetSpeed = -config().get<float>("velocityApproaching", 0.5)
                        +  (1.0-deltaX/brakingDistance) * deltaV;
            }
            else
            { */
      drive_command.lin_vel = -cfg_.velocity_approaching;
      //}

    }else{
      //begin steering into parking space
      drive_command.lin_vel = -cfg_.velocty_entering;

      if (! yawAngleSet)
      {
        yawAngleSet = true;
        yawAngleStartEntering = car_yawAngle; //set yawAngle reference
      }

      double drivenArc = car_yawAngle - yawAngleStartEntering;

      if (firstCircleArc) {
        // set servos to max steering angle
        drive_command.phi_f = -delta_max;
        drive_command.phi_r = delta_max;
        ROS_INFO_STREAM_NAMED(stream_name_, "Driven arc "<<drivenArc<<" target: "<<alpha);
        if (drivenArc >= alpha) {
          firstCircleArc = false;
        }
      }
      else {
        // set servos to max steering angle in other direction
        drive_command.phi_f = delta_max;
        drive_command.phi_r = -delta_max;

        if (drivenArc <= 0.0 + cfg_.alpha_offset) {
          drive_command.phi_f = 0.0; // * 180. / M_PI;
          drive_command.phi_r = 0.0; // * 180. / M_PI;
          drive_command.lin_vel = 0.0; //config().get<float>("velocityCorrecting", 0.5);

          currentXPosition = 0;
          currentState = ParkingState::CORRECTING;
        }
      }

    }

    break;

  }
  case ParkingState::CORRECTING:
  {
	  ROS_INFO("ParkingState::CORRECTING");
	  drive_command.blink_com = drive_command.NO_BLINK;

	  updatePositionFromHall();

	  std::vector<float> correctingDistances(1, 0.2f); // TODO:
	  double velocityCorrection = 0.5; // TODO: in config

	  if (correctingCounter >= (int)correctingDistances.size()) {
		  currentState = ParkingState::FINISHED;
	  } else {
		  double phi_ist = car_yawAngle - yawAngleStartEntering;

		  if((correctingCounter % 2) == 0) //forward
		  {

			  drive_command.lin_vel = velocityCorrection;
			  setSteeringAngles(drive_command, -0.29, 0.0, 0.0, 0.0*phi_ist, DrivingMode::FORWARD);

			  ROS_DEBUG("forward Correcting");
			  //mit dem lidar den Frontabstand Messen
			  float correctingDistance = correctingDistances.at(correctingCounter);
			  // TODO: this doesnt work
//			  if(false && validDistanceToObstacleFront){
//				  logger.debug("distanceToObstacleFront")<<distanceToObstacleFront;
//				  correctingDistance = distanceToObstacleFront-config().get<float>("distanceToObstacleFront",0.28);
//				  logger.debug("correctingDistance")<<correctingDistance<<" currentXPosition "<<currentXPosition;
//				  if(correctingDistance < 0){
//					  correctingCounter++;
//					  currentXPosition = 0.0;
//				  }
//			  } else
			  {
				  ROS_DEBUG_STREAM("no validDistanceToObstacleFront found " << distanceToObstacleFront);
				  if (currentXPosition >= correctingDistance) {
					  ++correctingCounter;
					  currentXPosition = 0.0;
				  }
			  }

		  }
		  else // backwards
		  {
			  drive_command.lin_vel = -1.0 * velocityCorrection;
			  setSteeringAngles(drive_command, -0.29, 0.0, 0.0, 0.0*phi_ist, DrivingMode::BACKWARDS);

			  ROS_DEBUG("backwards Correcting");
			  float correctingDistance = correctingDistances.at(correctingCounter);
			  // TODO: this doesnt work
//			  if(false && validDistanceToObstacleFront){
//				  logger.debug("distanceToObstacleFront")<<distanceToObstacleFront;
//				  correctingDistance = 0.53-distanceToObstacleFront-config().get<float>("distanceToObstacleRear",0.1);
//				  logger.debug("correctingDistance")<<correctingDistance<<" currentXPosition "<<currentXPosition;
//				  if(correctingDistance < 0){
//					  correctingCounter++;
//					  currentXPosition = 0.0;
//				  }
//			  }else
			  {
				  ROS_DEBUG_STREAM("no validDistanceToObstacleFront found" << distanceToObstacleFront);
				  if (-currentXPosition >= correctingDistance)
				  {
					  ++correctingCounter;
					  currentXPosition = 0.0;
				  }
			  }
		  }

	  }

	  break;
  }
  case ParkingState::FINISHED: {
	  ROS_DEBUG("FINISHED");

	  drive_command.lin_vel = 0.0;
	  drive_command.phi_f = 0.0;
	  drive_command.phi_r = 0.0;

	  updatePositionFromHall();

	  // we are done
	  if (finishCounter > cfg_.wait_in_finished)
	  {
		  drive_command.blink_com = drive_command.NO_BLINK;
		  currentState = ParkingState::PULLOUT;
		  car_yawAngle = 0;
		  finished_pos = currentXPosition;

	  }
	  else
	  {
		  drive_command.blink_com = drive_command.BLINK_BOTH;
	  }

	  ++finishCounter;


	  break;
  }
  case ParkingState::WORST_CASE_BACKWARDS: {
	  ROS_DEBUG("WORST_CASE_BACKWARDS");
	  // TODO

	  break;
  }
  case ParkingState::PULLOUT: {
	  ROS_DEBUG("PULL_OUT");

	  updatePositionFromHall();

	  switch(pulloutstate)
	  {

	  case PullOutState::PULLBACK:{

		  drive_command.lin_vel = cfg_.velocity_pullback;
		  drive_command.phi_f = 0.0;
		  drive_command.phi_r = 0.0;

		  float luecken_factor = (parkingSpaceSize - cfg_.min_parking_space_size)/((cfg_.max_parking_space_size) - cfg_.min_parking_space_size);

		  if(finished_pos - luecken_factor*cfg_.drive_backwards_dynamic - cfg_.drive_backwards > currentXPosition)
		  {
			  //state.targetSpeed = 0.0;
			  pulloutstate = PullOutState::TURN_LEFT;
		  }

		  break;
	  }
	  case PullOutState::TURN_LEFT:{
		  drive_command.lin_vel = cfg_.velocity_pullout_turn;
		  drive_command.phi_f = cfg_.max_steering_angle;
		  drive_command.phi_r = -1.0 * cfg_.max_steering_angle;
		  move_straight_start_pos = currentXPosition;
		  if(car_yawAngle >= cfg_.turn_first_yaw)
		  {
			  pulloutstate=PullOutState::MOVE_STRAIGHT;
		  }
		  break;
	  }
	  case PullOutState::MOVE_STRAIGHT:{
		  drive_command.lin_vel = cfg_.velocity_move_straight;
		  drive_command.phi_f = 0.0;
		  drive_command.phi_r = 0.0;
		  if(move_straight_start_pos + cfg_.move_straight_dist < currentXPosition){
			  pulloutstate=PullOutState::TURN_RIGHT;
		  }
		  break;

	  }
	  case PullOutState::TURN_RIGHT:{
		  drive_command.lin_vel = cfg_.velocity_pullout_turn;
		  drive_command.phi_f = -1.0 * cfg_.max_steering_angle;
		  drive_command.phi_r = -1.0 * cfg_.max_steering_angle;
		  if(car_yawAngle <= cfg_.turn_second_yaw)
		  {
			  pulloutstate=PullOutState::BACK_ON_TRACK;
		  }
		  break;
	  }
	  case PullOutState::BACK_ON_TRACK: {

		  // TODO: send "parking_done" command or something else to BT or whatever
		  //state.targetSpeed = 0;
		  //state.steering_front = 0;
		  //state.steering_rear = 0;

      done = true;
		  break;
	  } // case BACK_ON_TRACK
	  } // switch PullOutState::BACK_ON_TRACK
  } // case ParkingState::PULLOUT
#endif
  } // switch current state


  drive_command_pub_.publish(drive_command);
  return done;

} // cycle


//bool Parking::checkForGap() {
//  auto size = parking->size;
//  // old gap data
//  if(timeSpaceWasFound == parking->timestamp())
//  {
//    return false;
//  }
//
//  timeSpaceWasFound = parking->timestamp();
//
//  if(size > cfg_.min_parking_space_size && size < cfg_.max_parking_space_size)
//  {
//    ROS_INFO_STREAM_NAMED(stream_name_,
//                          "[PARKING] valid parking space found! Size: " << size << ", at: " << parking->position);
//    posXGap = parking->position;
//    parkingSpaceSize = size;
//    return true;
//  } else {
//    ROS_INFO_STREAM_NAMED(stream_name_, "[PARKING] Invalid parking space found! Size: " << size << ", at: " << parking->position);
//  }
//  return false;
//}

void Parking::updatePositionFromHall()
{
//  if(sensors->hasSensor("HALL")) {
//    auto hall = sensors->sensor<sensor_utils::Odometer>("HALL");
//    auto dst = hall->distance.x();
//    currentXPosition += dst;
//  }else{
//    logger.warn("no hall distance given!");
//    std::string s = "";
//    for(auto a:*sensors){
//      s += a.first + ", ";
//    }
//    logger.warn("sensors given:")<<s;
//  }

	geometry_msgs::TransformStamped transform =
			tfBuffer.lookupTransform(movingFrame, ros::Time::now(), movingFrame, startParkingStamp, staticFrame);

	currentXPosition = transform.transform.translation.x;
	car_yawAngle = tf::getYaw(transform.transform.rotation);
}

void Parking::fitLineToMiddleLane(double *oM, double *oB)
{
  float computeAtX = 0.5;
  if(drivingLineReceived) {
    (*oB) = compute_driving_lane_at_location(currentDrivingLine, computeAtX);
    (*oM) = derive_polynomial_at_location (currentDrivingLine->polynom_params, currentDrivingLine->polynom_order, computeAtX);
  }
}


//current values are calculated with respect to the (straight) middle lane
void Parking::setSteeringAngles(drive_ros_uavcan::phoenix_msgs__NucDriveCommand &driveCmd, double y_soll, double phi_soll, int drivingMode)
{

  //eigenvalues [-2, -2]
  double R_forward[] = {2.000, 1.420, 2.000, 1.000};
  double F_forward[] = {2.000, 0.420, 2.000, 0.000};
  double R_backwards[] = {-2.000, 0.580, -2.000, 1.000};
  double F_backwards[] = {-2.000, -0.420, -2.000, 0.000};

  //find regression line y=mx+b through some points of the middle lane (not necessary but contributes to better stability in straight line performance)
  double m, b;
  fitLineToMiddleLane(&m, &b);

  //the current vehicle state is [y_ist; phi_ist]
  double y_ist = -b*cos(-atan(m));
  double phi_ist = -atan(m);

  // define the controller gain R and the pre-filter F such that: u = [delta_v; delta_h] = F*[y_soll; phi_soll] - R*[y_ist; phi_ist]
  double delta_v, delta_h;
  if (drivingMode == DrivingMode::FORWARD)
  {
    delta_v = F_forward[0]*y_soll + F_forward[1]*phi_soll - (R_forward[0]*y_ist + R_forward[1]*phi_ist);
    delta_h = F_forward[2]*y_soll + F_forward[3]*phi_soll - (R_forward[2]*y_ist + R_forward[3]*phi_ist);
  }
  else
  {
    delta_v = F_backwards[0]*y_soll + F_backwards[1]*phi_soll - (R_backwards[0]*y_ist + R_backwards[1]*phi_ist);
    delta_h = F_backwards[2]*y_soll + F_backwards[3]*phi_soll - (R_backwards[2]*y_ist + R_backwards[3]*phi_ist);
  }

  //set the desired steering angles
  driveCmd.phi_f = delta_v;
  driveCmd.phi_r = delta_h;
}

void Parking::setSteeringAngles(drive_ros_uavcan::phoenix_msgs__NucDriveCommand &driveCmd, double y_soll, double phi_soll, double y_ist, double phi_ist, int drivingMode)
{

  //eigenvalues [-2, -2]
  double R_forward[] = {2.000, 1.420, 2.000, 1.000};
  double F_forward[] = {2.000, 0.420, 2.000, 0.000};
  double R_backwards[] = {-2.000, 0.580, -2.000, 1.000};
  double F_backwards[] = {-2.000, -0.420, -2.000, 0.000};

  // define the controller gain R and the pre-filter F such that: u = [delta_v; delta_h] = F*[y_soll; phi_soll] - R*[y_ist; phi_ist]
  double delta_v, delta_h;
  if (drivingMode == DrivingMode::FORWARD)
  {
    delta_v = F_forward[0]*y_soll + F_forward[1]*phi_soll - (R_forward[0]*y_ist + R_forward[1]*phi_ist);
    delta_h = F_forward[2]*y_soll + F_forward[3]*phi_soll - (R_forward[2]*y_ist + R_forward[3]*phi_ist);
  }
  else
  {
    delta_v = F_backwards[0]*y_soll + F_backwards[1]*phi_soll - (R_backwards[0]*y_ist + R_backwards[1]*phi_ist);
    delta_h = F_backwards[2]*y_soll + F_backwards[3]*phi_soll - (R_backwards[2]*y_ist + R_backwards[3]*phi_ist);
  }

  //set the desired steering angles
  driveCmd.phi_f = delta_v;
  driveCmd.phi_r = delta_h;
}

double Parking::getDistanceToMiddleLane()
{
  double m, b;
  fitLineToMiddleLane(&m, &b);

  return b;
}


