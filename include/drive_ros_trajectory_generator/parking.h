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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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

//    bool checkForGap();
    void updatePositionFromHall();

    void fitLineToMiddleLane(double *oM, double *oB);
    void setSteeringAngles(drive_ros_uavcan::phoenix_msgs__NucDriveCommand &driveCmd, double y_soll, double phi_soll, int drivingMode);
    void setSteeringAngles(drive_ros_uavcan::phoenix_msgs__NucDriveCommand &driveCmd, double y_soll, double phi_soll, double y_ist, double phi_ist, int drivingMode);
    double getDistanceToMiddleLane();

    void scanCB(const sensor_msgs::LaserScanConstPtr &scan);
    void setReconfigure(const drive_ros_trajectory_generator::TrajectoryLineCreationConfig cfg);
//    void drivingLineToScanSyncCB(const drive_ros_msgs::DrivingLineConstPtr &driving_line,
//                                 const sensor_msgs::LaserScanConstPtr &scan);
    ros::Subscriber scan_sub_;
    ros::Publisher drive_command_pub_;
    std::string stream_name_ = "parking_controller";
    PullOutState pulloutstate;
    ParkingState currentState;
    float lr; //Radstand
    float l;  //Fahrzeuglänge
    float b; //Fahrzeugbreite
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

    bool measuringGap = false;
    ros::Time gapFoundAt;
    bool spaceFound = false;

    tf2_ros::TransformListener tfListener;
    tf2_ros::Buffer tfBuffer;
    std::string staticFrame = "odom"; // TODO: from param
    std::string movingFrame = "rear_axis_middle_ground"; // TODO: from param
    ros::Time startParkingStamp;

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
