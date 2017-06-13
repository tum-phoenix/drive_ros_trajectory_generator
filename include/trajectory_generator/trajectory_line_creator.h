#ifndef IMAGE_HINT_TRANSFORMER_H
#define IMAGE_HINT_TRANSFORMER_H

#include "lms/module.h"
#include "lms/math/polyline.h"
#include "street_environment/road.h"
#include "street_environment/car.h"
#include "trajectory_generator.h"
#include "street_environment/trajectory.h"
#include "street_environment/obstacle.h"
#include <ros/ros.h>
#include "trajectory_generator/trajectory_line_creator.h"

enum class LaneState{
    CLEAR,DANGEROUS,BLOCKED
};

namespace trajectory_generator{

class TrajectoryLineCreator {

public:
    TrajectoryLineCreator(ros::NodeHandle& pnh);
    bool init();
    bool cycle() override;
private:
    //ros stuff
    ros::NodeHandle pnh_;
    ros::Subscriber envObstacles_sub_;
    ros::Subscriber roadStates_sub_;
    ros::Subscriber road_sub_;
    ros::Subscriber car_sub_;
    void envobs_callback(const /*TODO*/& msg)
    void roadstates_callback(const /*TODO*/& msg)
    void road_callback(const /*TODO*/& msg)
    void car_callback(const /*TODO*/& msg)
    ros::Publisher line_pub_;
    ros::Publisher debugTrajectory_pub_;
    //old stuff
    street_environment::Trajectory simpleTrajectory(bool useSavety, float endVelocity);
    TrajectoryGenerator* generator;

    // for the velocity adjustement
    float curvatureAtLargeDistancePT1 = 0;
    float alphaPT1; // between 0 and 1. if 1 only cujectory_generator::trajectory_line_creatorrrent value

    lms::math::vertex2f interpolateRoadAtDistance(const float distanceIn);
    float targetVelocity();
    int counter = 0;

    float dt = 0.01;
    float t_sinceLastUpdate = 0;

    bool obstacleInSightGlobal = false;
    bool oneTrajGenerated = false;

    float d1_last = -0.2;

    Trajectory lastResult;

    /**
     * @brief getLaneState
     * @param tangDistance
     * @param rightSide
     * @param reason TODO hier wird der pointer aus einem shared pointer geholt!
     * @return
     */
    LaneState getLaneState(const float tangDistance,const bool rightSide,street_environment::EnvironmentObject** reason = nullptr);


    float distanceTang(street_environment::ObstaclePtr obstacle);

    float distanceOrth(street_environment::ObstaclePtr obstacle);
};

} // namespace trajectory_generator

#endif /* IMAGE_HINT_TRANSFORMER_H */
