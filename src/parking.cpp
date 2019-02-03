#include <drive_ros_trajectory_generator/parking.h>
#include <limits>

Parking::Parking(ros::NodeHandle &nh, ros::NodeHandle &pnh)
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

  scan_sub_ = nh_.subscribe("scan_in", 2, &Parking::scanCB, this);
  parking_spot_sub_ = nh_.subscribe("parking_spot_in", 2, &Parking::parkingSpotCB, this);
  drive_command_pub_ = nh_.advertise<drive_ros_uavcan::phoenix_msgs__NucDriveCommand>("can_topic", 5);

  pnh_.param<float>("wheel_base", lr, 0.23); //Radstand
  pnh_.param<float>("car_length", l, 0.39);  //Fahrzeuglänge
  pnh_.param<float>("car_width", l, 0.21); //Fahrzeugbreite
  pnh_.param<float>("car_width", l, 32); //maximum steering angle
  l = l*M_PI/180;

  currentXPosition = 0;
  lastTimeStamp = -1;
  lastImuTimeStamp = -1;

  move_straight_start_pos = 0;
  finished_pos = 0;
  oldDeltaPhi = 0;
  distanceToObstacleFront = 0;
}

void Parking::scanCB(const sensor_msgs::LaserScanConstPtr &scan){
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
        }

        if(added!= 0) {
            validDistanceToObstacleFront = true;
            distanceToObstacleFront = smallestDistance;
            ROS_INFO_STREAM_NAMED(stream_name_, "[PARKING] Distance to the obstacle up front "<<distanceToObstacleFront);
        }
    }else{
        ROS_WARN_STREAM_NAMED(stream_name_, "[PARKING] No lidar data given!");
    }
}

bool Parking::triggerParking() {
  ROS_INFO_NAMED(stream_name_, "[PARKING] Starting to park");

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

  drive_ros_uavcan::phoenix_msgs__NucDriveCommand drive_command;
  switch (currentState)
  {
  case ParkingState::SEARCHING:
  {

    // turn of indicators
    drive_command.blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::NO_BLINK;

    /***************************************************
         * drive straight along the middle of the right lane
         ***************************************************/


    // the desired state is such that phi=0 (straight driving) and y=0.2 (middle of right lane) with respect to the middle lane
    setSteeringAngles(cfg_.obstacle_detection_angle, cfg_.searching_angle, DrivingMode::FORWARD);

    drive_command.lin_vel = cfg_.velocity_searching;

    // check if we detected a valid parking space
    bool spaceFound = checkForGap();
    ROS_DEBUG_STREAM_NAMED(stream_name_, "[PARKING] Searching for gap, found "<<spaceFound);

    if (spaceFound) {
      currentState = ParkingState::STOPPING;
      startParkingStamp = ros::Time::now();
    }

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
    setSteeringAngles(cfg_.searching_middle_offset, cfg_.searching_phi_factor, DrivingMode::FORWARD);

    //update current x-position
    updatePositionFromHall();

    //set target speed as if the car was decelerating constantly
    if (cfg_.deceleration_stopping) {
      drive_command.lin_vel = cfg_.velocty_searching - (ros::Time::now()-startParkingStamp).toSec()*
              cfg_.deceleration_stopping;
    }
    else
    {
      drive_command.lin_vel = 0.0;
    }

    //int num_y_vals = config().get("numberOfY0measurements", 20);
    // todo: get velocity from TF here
    if (car->velocity() < cfg_.min_velocity_before_driving_backwards) {

      y0_dynamic = cfg_.y0_worstCase;

      currentState = ParkingState::ENTERING;
    }

    break;
  }
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
    logger.debug("CORRECTING");
    state.indicatorLeft = false;
    state.indicatorRight = false;

    updatePositionFromHall();

    std::vector<float> correctingDistances = config().getArray<float>("correctingDistances");

    if (correctingCounter >= (int)correctingDistances.size()){
      currentState = ParkingState::FINISHED;
    }else{
      double phi_ist = car_yawAngle - yawAngleStartEntering;

      if (correctingCounter%2 == 0) //forward
      {
        state.targetSpeed = config().get<float>("velocityCorrecting", 0.5);
        setSteeringAngles(-0.29, 0.0, 0.0, 0.0*phi_ist, DrivingMode::FORWARD);

        logger.debug("forward Correcting");
        //mit dem lidar den Frontabstand Messen
        float correctingDistance = correctingDistances.at(correctingCounter);
        // TODO: this doesnt work
        if(false && validDistanceToObstacleFront){
          logger.debug("distanceToObstacleFront")<<distanceToObstacleFront;
          correctingDistance = distanceToObstacleFront-config().get<float>("distanceToObstacleFront",0.28);
          logger.debug("correctingDistance")<<correctingDistance<<" currentXPosition "<<currentXPosition;
          if(correctingDistance < 0){
            correctingCounter++;
            currentXPosition = 0.0;
          }
        }else{
          logger.debug("no validDistanceToObstacleFront found")<<distanceToObstacleFront;
          if (currentXPosition >= correctingDistance){
            ++correctingCounter;
            currentXPosition = 0.0;
          }
        }

      }
      else //backwards
      {
        state.targetSpeed = -config().get<float>("velocityCorrecting", 0.5);
        setSteeringAngles(-0.29, 0.0, 0.0, 0.0*phi_ist, DrivingMode::BACKWARDS);

        logger.error("backwards Correcting");
        float correctingDistance = correctingDistances.at(correctingCounter);
        // TODO: this doesnt work
        if(false && validDistanceToObstacleFront){
          logger.debug("distanceToObstacleFront")<<distanceToObstacleFront;
          correctingDistance = 0.53-distanceToObstacleFront-config().get<float>("distanceToObstacleRear",0.1);
          logger.debug("correctingDistance")<<correctingDistance<<" currentXPosition "<<currentXPosition;
          if(correctingDistance < 0){
            correctingCounter++;
            currentXPosition = 0.0;
          }
        }else{
          logger.debug("no validDistanceToObstacleFront found")<<distanceToObstacleFront;
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
    logger.debug("FINISHED");

    state.targetSpeed = 0.0;
    state.steering_front = 0.0;
    state.steering_rear = 0.0;

    updatePositionFromHall();

    // we are done
    if (finishCounter > config().get<int>("waitInFinished", 50))
    {
      state.indicatorLeft = false;
      state.indicatorRight = false;
      currentState = ParkingState::PULLOUT;
      car_yawAngle = 0;
      finished_pos = currentXPosition;

    }
    else
    {
      state.indicatorLeft = true;
      state.indicatorRight = true;
    }

    ++finishCounter;


    break;
  }
  case ParkingState::WORST_CASE_BACKWARDS: {
    logger.debug("WORST_CASE_BACKWARDS");
    // TODO

    break;
  }
  case ParkingState::PULLOUT: {
    logger.debug("PULL_OUT");

    updatePositionFromHall();

    switch(pulloutstate)
    {

    case PullOutState::PULLBACK:{

      state.targetSpeed = config().get<float>("velocityPullback", -0.5);
      state.steering_front = 0;
      state.steering_rear = 0;

      float luecken_factor = (parkingSpaceSize - config().get("minParkingSpaceSize", 0.60))/(float)(config().get<float>("maxParkingSpaceSize", 0.80) - config().get<float>("minParkingSpaceSize", 0.60));

      if(finished_pos - luecken_factor*config().get<float>("driveBackwardsDynamic", 0.05) - config().get<float>("driveBackwards", 0.05) > currentXPosition)
      {
        //state.targetSpeed = 0.0;
        pulloutstate = PullOutState::TURN_LEFT;

      }

      break;
    }
    case PullOutState::TURN_LEFT:{
      state.targetSpeed = config().get<float>("velocityTurnLeft", 1);
      state.steering_front = config().get("maxSteeringAngle", 24);
      state.steering_rear = -config().get("maxSteeringAngle", 24);
      move_straight_start_pos = currentXPosition;
      if(car_yawAngle >= config().get<float>("turnYaw", 0.8))
      {
        pulloutstate=PullOutState::MOVE_STRAIGHT;
      }
      break;
    }
    case PullOutState::MOVE_STRAIGHT:{
      state.steering_front = 0;
      state.steering_rear = 0;
      state.targetSpeed = config().get<float>("velocityMoveStraight", 1);
      if(move_straight_start_pos + config().get<float>("moveStraight", 0.1) < currentXPosition){
        pulloutstate=PullOutState::TURN_RIGHT;
      }
      break;

    }
    case PullOutState::TURN_RIGHT:{
      state.targetSpeed = config().get<float>("velocityTurnRight", 1);;
      state.steering_front = -config().get("maxSteeringAngle", 24);
      state.steering_rear = config().get("maxSteeringAngle", 24);
      if(car_yawAngle <= config().get<float>("turnYaw2", 0.2))
      {
        pulloutstate=PullOutState::BACK_ON_TRACK;
      }
      break;
    }
    case PullOutState::BACK_ON_TRACK: {

      // switch mode back to FOH
      getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE")->setDriveMode(phoenix_CC2016_service::CCDriveMode::PARKING_END);
      //state.targetSpeed = 0;
      //state.steering_front = 0;
      //state.steering_rear = 0;

      break;
    } // case BACK_ON_TRACK
    } // switch PullOutState::BACK_ON_TRACK
  } // case ParkingState::PULLOUT
  } // switch current state

  car->putState(state);
  return true;

} // cycle


bool Parking::checkForGap()
{
  auto size = parking->size;
  // old gap data
  if(timeSpaceWasFound == parking->timestamp())
  {
    return false;
  }

  timeSpaceWasFound = parking->timestamp();

  if(size > cfg_.min_parking_space_size && size < cfg_.max_parking_space_size)
  {
    ROS_INFO_STREAM_NAMED(stream_name_,
                          "[PARKING] valid parking space found! Size: " << size << ", at: " << parking->position);
    posXGap = parking->position;
    parkingSpaceSize = size;
    return true;
  } else {
    ROS_INFO_STREAM(stream_name_, "[PARKING] Invalid parking space found! Size: " << size << ", at: " << parking->position;
  }
  return false;
}

void Parking::updatePositionFromHall()
{
  if(sensors->hasSensor("HALL")) {
    auto hall = sensors->sensor<sensor_utils::Odometer>("HALL");
    auto dst = hall->distance.x();
    currentXPosition += dst;
  }else{
    logger.warn("no hall distance given!");
    std::string s = "";
    for(auto a:*sensors){
      s += a.first + ", ";
    }
    logger.warn("sensors given:")<<s;
  }
}

void Parking::fitLineToMiddleLane(double *oM, double *oB)
{
  //get points from middle lane
  street_environment::RoadLane middleLane = getService<local_course::LocalCourse>("LOCAL_COURSE_SERVICE")->getCourse();
  std::vector<double> iX;
  std::vector<double> iY;
  int lineFitStartPoint = config().get<int>("lineFitStartPoint", 2);
  int lineFitEndPoint = config().get<int>("lineFitEndPoint", 5);
  if(middleLane.points().size() <= lineFitEndPoint){
    logger.error("invalid lineFitEndPoint")<<lineFitEndPoint<<" ,middle has only "<<middleLane.points().size()<<" elements";
    lineFitEndPoint = ((int)middleLane.points().size()) -1;
  }
  for (int i=lineFitStartPoint; i<=lineFitEndPoint; ++i) //get points 1 to 4
  {
    iX.push_back(middleLane.points().at(i).x);
    iY.push_back(middleLane.points().at(i).y);
  }

  if (iX.size() != iY.size())
  {
    *oM = 0.0;
    *oB = 0.0;
    return;
  }

  double xMean = 0.0, yMean = 0.0;
  for (uint i=0; i < iX.size(); ++i)
  {
    xMean += iX.at(i);
    yMean += iY.at(i);
  }
  xMean /= iX.size();
  yMean /= iX.size();

  double nom = 0.0, den = 0.0;
  for (uint i=0; i < iX.size(); ++i)
  {
    nom += (iX.at(i)-xMean)*(iY.at(i)-yMean);
    den += (iX.at(i)-xMean)*(iX.at(i)-xMean);
  }

  // regression line: y = m*x + b
  *oM = nom/den; //slope of line
  *oB = yMean - *oM*xMean; //y-intercept

  return;
}


//current values are calculated with respect to the (straight) middle lane
void Parking::setSteeringAngles(double y_soll, double phi_soll, int drivingMode)
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
  state.steering_front = delta_v;
  state.steering_rear = delta_h;

}

void Parking::setSteeringAngles(double y_soll, double phi_soll, double y_ist, double phi_ist, int drivingMode)
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
  state.steering_front = delta_v;
  state.steering_rear = delta_h;

}

double Parking::getDistanceToMiddleLane()
{
  double m, b;
  fitLineToMiddleLane(&m, &b);

  return b;
}


