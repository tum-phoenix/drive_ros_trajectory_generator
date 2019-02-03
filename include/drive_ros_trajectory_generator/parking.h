#ifndef PARKING_H
#define PARKING_H

#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <float.h>
#include <drive_ros_uavcan/phoenix_msgs__ParallelParking.h>
#include <drive_ros_uavcan/phoenix_msgs__NucDriveCommand.h>
#include <drive_ros_msgs/DrivingLine.h>
#include <sensor_msgs/LaserScan.h>
#include "drive_ros_trajectory_generator/TrajectoryLineCreationConfig.h"
#include <drive_ros_msgs/EnvironmentModel.h>

class Parking {
public:
    Parking(ros::NodeHandle &nh, ros::NodeHandle &pnh);

    enum DrivingMode {FORWARD, BACKWARDS};
    enum ParkingState {SEARCHING, STOPPING, ENTERING, CORRECTING, FINISHED, WORST_CASE_BACKWARDS, PULLOUT};
    enum PullOutState {PULLBACK, TURN_LEFT, MOVE_STRAIGHT,  TURN_RIGHT, BACK_ON_TRACK};

    struct ParkingStateContainer{
        ParkingState state;
        DrivingMode driveMode;
    };

public:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    bool triggerParking();

    bool checkForGap();
    void updatePositionFromHall();

    void fitLineToMiddleLane(double *oM, double *oB);
    void setSteeringAngles(double y_soll, double phi_soll, int drivingMode);
    void setSteeringAngles(double y_soll, double phi_soll, double y_ist, double phi_ist, int drivingMode);
    double getDistanceToMiddleLane();

    void scanCB(const sensor_msgs::LaserScanConstPtr &scan);
    void parkingSpotCB(const drive_ros_uavcan::phoenix_msgs__ParallelParking &parking_gap);
    void drivingLineToScanSyncCB(const drive_ros_msgs::DrivingLineConstPtr &driving_line,
                                 const sensor_msgs::LaserScanConstPtr &scan);
    ros::Time startParkingStamp;
    ros::Subscriber parking_spot_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher drive_command_pub_;
    std::string stream_name_ = "parking_controller";
    PullOutState pulloutstate;
    ParkingState currentState;
    double lr; //Radstand
    double l;  //Fahrzeuglänge
    double b; //Fahrzeugbreite
    double delta_max; //maximum steering angle
    bool firstCircleArc;
    double lastTimeStamp, lastImuTimeStamp, currentXPosition;
    double y0_dynamic, ind_end, endX;
    ros::Time timeSpaceWasFound;
    bool straightMove;
    bool yawAngleSet;
    int finishCounter;
    drive_ros_trajectory_generator::TrajectoryLineCreationConfig cfg_;

    double car_yawAngle, car_xPosition;
    double yawAngleStartEntering;
    int correctingCounter;

    double posXGap;
    double parkingSpaceSize;

    // asynchronously filled by callback
    float distanceToObstacleFront;

    // hack
    int m_cycleCounter;
    double move_straight_start_pos;
    double finished_pos;
    float initial_phi;

    double oldDeltaPhi;

};

#endif // PARKING_H
