#ifndef IMAGE_HINT_TRANSFORMER_H
#define IMAGE_HINT_TRANSFORMER_H

//#include "lms/module.h"
//#include "lms/math/polyline.h"
//#include "street_environment/road.h"
//#include "street_environment/car.h"
//#include "trajectory_generator.h"
//#include "street_environment/trajectory.h"
//#include "street_environment/obstacle.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "trajectory_generator/trajectory_line_creator.h"

enum class LaneState{
    CLEAR,DANGEROUS,BLOCKED
};

namespace trajectory_generator{

class TrajectoryLineCreator {

public:
    TrajectoryLineCreator(const ros::NodeHandle nh, const ros::NodeHandle pnh);
    bool init();
private:
    //ros stuff
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber costmap_sub_;
    void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    ros::Publisher trajectory_pub_;
    ros::Publisher debugTrajectory_pub_;

    geometry_msgs::Pose* trajectory;
    geometry_msgs::Pose* debug_trajectory;


    //commented out old stuff in case it is needed

    //new stuff probably not needed
    /*street_environment::EnvironmentObjects* envObstacles;
    street_environment::RoadStates* roadStates;
    street_environment::RoadLane* road;
    street_environment::CarCommand* car;*/

    /*//old stuff
    street_environment::Trajectory simpleTrajectory(bool useSavety, float endVelocity);
    TrajectoryGenerator* generator;

    // for the velocity adjustement
    float curvatureAtLargeDistancePT1 = 0;
    float alphaPT1; // between 0 and 1. if 1 only cujectory_generator::trajectory_line_creatorrrent value

    // TODO lms::math::vertex2f interpolateRoadAtDistance(const float distanceIn);
    float targetVelocity();
    int counter = 0;

    float dt = 0.01;
    float t_sinceLastUpdate = 0;

    bool obstacleInSightGlobal = false;
    bool oneTrajGenerated = false;

    float d1_last = -0.2;

    // TODO Trajectory lastResult;

    *//**
     * @brief getLaneState
     * @param tangDistance
     * @param rightSide
     * @param reason TODO hier wird der pointer aus einem shared pointer geholt!
     * @return
     *//*
    LaneState getLaneState(const float tangDistance,const bool rightSide,street_environment::EnvironmentObject** reason = nullptr);


    float distanceTang(street_environment::ObstaclePtr obstacle);

    float distanceOrth(street_environment::ObstaclePtr obstacle);*/
};

} // namespace trajectory_generator

#endif /* IMAGE_HINT_TRANSFORMER_H */
