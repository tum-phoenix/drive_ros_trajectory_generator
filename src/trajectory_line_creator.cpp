//#include "lms/math/math.h"
//#include "street_environment/obstacle.h"
//#include "street_environment/crossing.h"
//#include "lms/math/mathEigen.h"
//#include "phoenix_CC2016_service/phoenix_CC2016_service.h"
#include <trajectory_generator/trajectory_line_creator.h>
#include <ros/node_handle.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
//#include <street_environment/trajectory.h>

namespace trajectory_generator {

    TrajectoryLineCreator::TrajectoryLineCreator(const ros::NodeHandle nh, const ros::NodeHandle pnh) :
            pnh_(pnh),
            costmap_sub_() {
        //advertise to publish topics TODO auch in init()?
        trajectory_pub_ = pnh_.advertise<geometry_msgs::Pose2D>("LINE", 1); //old name: trajectory
        debugTrajectory_pub_ = pnh_.advertise<geometry_msgs::Pose2D>("DEBUG_TRAJECTORY", 1); //old name: debug_trajectory

        trajectory = new geometry_msgs::Pose(); //TODO hier ::ConstPtr ?
    }

    bool TrajectoryLineCreator::init() {
        //TODO pnh_.getparam()

        //subscribe to all needed topics
        costmap_sub_ = pnh_.subscribe("costmap_in", 100, &TrajectoryLineCreator::costmap_callback, this);

        return true;
    }

    void TrajectoryLineCreator::costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
        //TODO set costmap
        // for invoking test callback:
        // andi@andi-phoenix:~$ rostopic pub -1 /trajectory_line_creator_node/costmap_in
        // nav_msgs/OccupancyGrid -- '{header: auto, info: {map_load_time: now,
        // resolution: 1.2, width: 23, height: 23}, data: [6, 8]}'
        geometry_msgs::Pose2D pose;
        pose.x = msg->info.resolution;
        pose.theta = 0.2;
        debugTrajectory_pub_.publish(pose);
    }

    /* start of Legacy Code
    bool TrajectoryLineCreator::cycle() { //TODO where is ros-cycle? publish?
        //clear old trajectory
        trajectory->clear();
        debug_trajectory->points().clear();

        //calculate the speed without obstacles
        float velocity = targetVelocity();
        //calculate the trajectory
        *trajectory = simpleTrajectory(true, velocity);

        for (street_environment::TrajectoryPoint &v:*trajectory) {
            debug_trajectory->points().push_back(v.position);
        }

        return true;
    }

    float TrajectoryLineCreator::targetVelocity() {
        if (roadStates->states.size() != 3) {
            ROS_ERROR("Error in targetVelocity(): invalid roadstates given, size: %l" , roadStates->states.size()); //TODO richtig angezeigt?
            return 0;
        }
        const float obstacleTrustThreshold = config().get<float>("obstacleTrustThreshold", 0.5);
        float velocity = 0;
        bool obstacleInSight = false;
        float distanceToObstacle = 0;
        for (street_environment::EnvironmentObjectPtr obj:envObstacles->objects) {
            if (obj->getType() != street_environment::Obstacle::TYPE)
                continue;
            street_environment::ObstaclePtr obst = std::static_pointer_cast<street_environment::Obstacle>(obj);
            //Only looking for obstacles on the right side
            if (distanceOrth(obst) < 0.1 && obst->trust() > obstacleTrustThreshold) {
                obstacleInSight = true;
                if (obst->position().x > -0.1 && (distanceToObstacle > distanceTang(obst))) {
                    distanceToObstacle = distanceTang(obst);
                }
            }
        }

        Eigen::Vector3f stateVelocities;

        //TODO change to ROS
        const lms::Config* myConfig = &config();
        lms::ServiceHandle <phoenix_CC2016_service::Phoenix_CC2016Service> service = phoenix_CC2016_service::Phoenix_CC2016Service::getService<phoenix_CC2016_service::Phoenix_CC2016Service>(
                "PHOENIX_SERVICE");

        if (!service.isValid()) {
            ROS_ERROR("PHOENIX SERVICE IS INVALID!");
        } else {
            if (service->driveMode() == phoenix_CC2016_service::CCDriveMode::FMH) {
                myConfig = &config("FMH");
            } else if (service->driveMode() == phoenix_CC2016_service::CCDriveMode::FOH) {
                myConfig = &config("FOH");
            } else if (service->driveMode() == phoenix_CC2016_service::CCDriveMode::PARKING) {
                //we do nothing
            } else if (service->driveMode() == phoenix_CC2016_service::CCDriveMode::IDLE) {
                //we do nothing
            } else {
                ROS_WARN("no drivemode set! Using default config");
            }
        }

        stateVelocities(0) = myConfig->get<float>("velocity_straight", 6);

        float aOrthMax = myConfig->get<float>("aOrthMax", 9.81 * 0.5);
        float curve_minVelocity = myConfig->get<float>("curve_minVelocity", 1.8);
        float curve_maxVelocity = myConfig->get<float>("curve_maxVelocity", 5);
        float curveStraight_minVelocity = myConfig->get<float>("curveStraight_minVelocity", 1.5);
        float curveStraight_maxVelocity = myConfig->get<float>("curveStraight_maxVelocity", 2.0);

        //calculate the velocity given by the roadStates
        float straightCurveVelocity = sqrt(aOrthMax / fabs(roadStates->states[1].curvature));
        float curveVelocity = sqrt(aOrthMax / fabs(roadStates->states[2].curvature));

        if (roadStates->states[2].curvature == 0) {
            curveVelocity = curve_maxVelocity;
        }
        if (roadStates->states[1].curvature == 0) {
            straightCurveVelocity = straightCurveVelocity;
        }


        if (curveVelocity < curve_minVelocity) {
            curveVelocity = curve_minVelocity;
        }
        if (curveVelocity > curve_maxVelocity) {
            curveVelocity = curve_maxVelocity;
        }
        if (straightCurveVelocity < curveStraight_minVelocity) {
            straightCurveVelocity = curveStraight_minVelocity;
        }
        if (straightCurveVelocity > curveStraight_maxVelocity) {
            straightCurveVelocity = curveStraight_maxVelocity;
        }

        stateVelocities(1) = straightCurveVelocity;
        stateVelocities(2) = curveVelocity;

        Eigen::Vector3f stateProbabilities;
        stateProbabilities(0) = roadStates->states[0].probability;
        stateProbabilities(1) = roadStates->states[1].probability;
        stateProbabilities(2) = roadStates->states[2].probability;

        velocity = (stateProbabilities.cwiseProduct(stateVelocities)).sum() / stateProbabilities.sum();

        if (velocity > stateVelocities(0)) {
            velocity = stateVelocities(0);
        }

        if (velocity < curve_minVelocity) {
            velocity = curve_minVelocity;
        }

        if (obstacleInSight) {
            velocity = velocity * config().get<float>("obstacleVelocitySafetyFactor", 0.65);
        } else {
            //do nothing
        }

        return velocity;
    }


    street_environment::Trajectory TrajectoryLineCreator::simpleTrajectory(bool useSavety, float endVelocity) {
        bool useFixedSpeed = false;
        float fixedSpeed = 0;
        bool firstAdd = true;
        //TODO change to ROS
        lms::ServiceHandle <phoenix_CC2016_service::Phoenix_CC2016Service> phoenixService = getService<phoenix_CC2016_service::Phoenix_CC2016Service>(
                "PHOENIX_SERVICE");

        street_environment::Trajectory tempTrajectory;
        // translate the middle lane to the right with a quarter of the street width
        const float translation = config().get<float>("street_width", 0.8) / 4.0f;

        //TODO change to ROS
        using lms::math::vertex2f;
        if (road->points().size() == 0) {
            ROS_ERROR("cycle: no valid environment given");
            return tempTrajectory;
        }
        //check if road is valid //TODO change to ROS
        for (const lms::math::vertex2f &v:road->points()) {
            if (std::isnan(v.x) || std::isnan(v.y)) {
                ROS_ERROR("simpleTrajectory: Invalid road, element is nan!");
                return tempTrajectory;
            }
        }
        const float trajectoryStartDistance = config().get<float>("trajectoryStartDistance", 0.3);
        const float obstacleResolution = config().get<float>("obstacleResolution", 0.05);
        //TODO change to ROS
        const lms::math::polyLine2f middle = road->getWithDistanceBetweenPoints(obstacleResolution);
        float tangLength = 0;
        //ignore first point
        for (int i = 1; i < (int) middle.points().size(); i++) {
            const vertex2f p1 = middle.points()[i - 1];
            const vertex2f p2 = middle.points()[i];
            if (p1 == p2) { //should never happen
                ROS_ERROR("simpleTrajectory: p1 == p2");
                continue;
            }

            const vertex2f along = p2 - p1;
            //check if the trajectory is long enough
            //TODO, get endpoint
            tangLength += along.length();
            if (tangLength < trajectoryStartDistance *//* tangLength > trajectoryMaxLength*//*) {//TODO trajectoryMaxLength
                continue;
            }
            const vertex2f normAlong = along / along.length();
            const vertex2f orthogonal(normAlong.y, -normAlong.x);
            const vertex2f orthogonalTrans = orthogonal * translation;

            bool rightSide = true;


            street_environment::EnvironmentObject *reasonObj;
            if (!(useFixedSpeed && (fixedSpeed == 0))) {
                //check all obstacles
                LaneState rightState = LaneState::CLEAR;
                LaneState leftState = LaneState::CLEAR;
                if (phoenixService->driveMode() == phoenix_CC2016_service::CCDriveMode::FMH) {
                    ROS_DEBUG("simpleTrajectory: driving with obstacles");
                    rightState = getLaneState(tangLength, true, &reasonObj);
                    leftState = getLaneState(tangLength, false, &reasonObj);
                    //if useSavety, we will avoid the right lane if it's blocked/dangerous
                    if (rightState > leftState && (useSavety || (rightState == LaneState::BLOCKED))) {
                        rightSide = false; //if both are blocked, we will stay on the right
                    }
                } else if (phoenixService->driveMode() == phoenix_CC2016_service::CCDriveMode::FOH) {
                    ROS_DEBUG("simpleTrajectory: driving without obstacles");
                } else {
                    ROS_DEBUG("simpleTrajectory: no valid mode set, using driving without obstacles: "
                                                     ,(int) phoenixService->driveMode());
                }
                ROS_DEBUG("states", (int) rightState, " ", (int) leftState);
                //check if the road is blocked, if yes, stop
                if (rightSide && rightState == LaneState::BLOCKED) {
                    if (reasonObj != nullptr && reasonObj->getType() == street_environment::Obstacle::TYPE) { //TODO
                        ROS_INFO("street is blocked by obstacles!");
                    } else {
                        ROS_INFO("street is blocked by crossing!");
                    }
                    useFixedSpeed = true;
                    fixedSpeed = 0;
                }
            }

            if (useFixedSpeed) {
                endVelocity = fixedSpeed;
            }
            vertex2f result;
            if (rightSide) {
                if (firstAdd) {
                    firstAdd = false;
                    //TODO change to ROS
                    tempTrajectory.push_back(
                            street_environment::TrajectoryPoint(lms::math::vertex2f(0, 0), normAlong, endVelocity,
                                                                -0.2)); //TODO
                }
                result = p1 + orthogonalTrans;
                tempTrajectory.push_back(
                        street_environment::TrajectoryPoint(result, normAlong, endVelocity, -0.2)); //TODO
            } else {
                if (firstAdd) {
                    firstAdd = false;
                    //TODO change to ROS
                    tempTrajectory.push_back(
                            street_environment::TrajectoryPoint(lms::math::vertex2f(0, 0), normAlong, endVelocity,
                                                                0.2)); //TODO
                }
                result = p1 - orthogonalTrans;
                tempTrajectory.push_back(
                        street_environment::TrajectoryPoint(result, normAlong, endVelocity, 0.2)); //TODO
            }
        }

        return tempTrajectory;

    }


    LaneState TrajectoryLineCreator::getLaneState(const float tangDistanceLane, const bool rightSide,
                                                  street_environment::EnvironmentObject **reason) {
        //Mindestabstand zwischen zwei Hindernissen 1m
        //Maximalabstand von der Kreuzung: 15cm
        //An der Kreuzung warten: 3s
        LaneState result = LaneState::CLEAR;
        const float obstacleTrustThreshold = config().get<float>("obstacleTrustThreshold", 0.5);
        const float obstacleLength = config().get<float>("obstacleLength", 0.3);
        const float obstacleSavetyDistance = config().get<float>("obstacleSavetyDistance", 0);
        const float crossingTrustThreshold = config().get<float>("crossingTrustThreshold", 0.5);
        const float obstacleMaxOrthDistance = config().get<float>("obstacleMaxOrthDistance", 0.35);
        //TODO if there is an obstacle close to the crossing we might fail
        //TODO we should sort everything by their tang distance
        //check if there is a crossing
        for (street_environment::EnvironmentObjectPtr objPtr: envObstacles->objects) {
            if (objPtr->getType() == street_environment::Crossing::TYPE) {
                ROS_DEBUG("I HAVE A CROSSING", objPtr->trust());
                //Check the trust
                if (objPtr->trust() < crossingTrustThreshold) {
                    continue;
                }
                const street_environment::CrossingPtr crossing = std::static_pointer_cast<street_environment::Crossing>(
                        objPtr);
                ROS_DEBUG("CROSSING DATA", distanceTang(crossing), " pos: ", crossing->position());
                if (crossing->position().x <
                    config().get<float>("crossingMinDistance", 0.3)) { //we already missed the trajectory!
                    continue;
                }
                //TODO doesn't work that good if(crossing->foundOppositeStopLine || !config().get<bool>("crossingUseOppositeLine",false)){
                if (car->velocity() < 0.1) {//TODO HACK but may work
                    if (const_cast<street_environment::Crossing *>(crossing.get())->startStop()) {//TODO HACK
                        ROS_INFO("start waiting in front of crossing");
                    }
                }
                ROS_INFO("simpleTrajectory", "crossing: stop ", crossing->hasToStop(), " blocked:"
                                                , crossing->blocked());
                if (crossing->hasWaited()) {
                    ROS_INFO("simpleTrajectory", " waiting for:", crossing->stopTime().since().toFloat());
                } else {
                    ROS_DEBUG(("simpleTrajectory"), " haven't waited on crossing yet");
                }

                //check if we have to stop or if crossing is blocked
                if (crossing->hasToStop() || crossing->blocked()) {
                    //Check if we are waiting for to long
                    if (crossing->hasWaited() &&
                        crossing->stopTime().since().toFloat() > config().get<float>("maxStopTimeAtCrossing", 10)) {
                        ROS_WARN("ignoring crossing", "I was waiting for ", crossing->stopTime().since(), "s");
                    } else {
                        //check if the Crossing is close enough
                        //As there won't be an obstacle in front of the crossing we can go on the right
                        //TODO we won't indicate if we change line
                        ROS_DEBUG("tang distance to crossing", distanceTang(crossing) - tangDistanceLane);
                        if (distanceTang(crossing) - tangDistanceLane <
                            config().get<float>("minDistanceToCrossing", 0.1)) {
                            result = LaneState::BLOCKED;
                            if (reason != nullptr) {
                                *reason = objPtr.get();
                            }
                            return result; //we have a crossing, we will drive and stop on the right
                        }
                    }
                } else {
                    ROS_INFO("CROSSING", "clear, I will go on :)");
                }
            }
        }
        //check if there is a obstacle
        for (street_environment::EnvironmentObjectPtr objPtr: envObstacles->objects) {
            if (objPtr->getType() == street_environment::Obstacle::TYPE) {
                if (objPtr->trust() < obstacleTrustThreshold) {
                    continue;
                }
                street_environment::ObstaclePtr obstPtr = std::static_pointer_cast<street_environment::Obstacle>(
                        objPtr);
                float orthDistanceToObstacle = distanceOrth(obstPtr);
                if (std::fabs(orthDistanceToObstacle) > obstacleMaxOrthDistance) {
                    continue;
                }

                //check if the obstacle is on the side we are looking for
                if (!((rightSide && orthDistanceToObstacle < 0) || (!rightSide && orthDistanceToObstacle > 0))) {
                    continue;
                }

                float distanceToObstacle = tangDistanceLane - distanceTang(obstPtr);//abstand zum Punkt p2
                if (obstPtr->trust() < obstacleTrustThreshold) {
                    continue;
                }
                if (distanceToObstacle < obstacleLength) {
                    if (reason != nullptr) {
                        *reason = objPtr.get();
                    }
                    if (distanceToObstacle >= 0 && distanceToObstacle) {
                        result = LaneState::BLOCKED;
                        break;
                    } else if (distanceToObstacle >= -obstacleSavetyDistance) {
                        result = LaneState::DANGEROUS;
                    }
                }
                *//*
                if(!rightSide && (int)result > 0){
                    ROS_ERROR("states pos: ")<<obstPtr->position().x<<" "<<obstPtr->position().y<<" dist: "<<tangDistanceLane<<" "<<distanceToObstacle;
                }
                *//*
            }
        }
        return result;
    }

    float TrajectoryLineCreator::distanceTang(street_environment::ObstaclePtr obstacle) {
        float t, o;
        road->firstOrthogonalDistance(obstacle->position(), o, t);
        return t;
    }

    float TrajectoryLineCreator::distanceOrth(street_environment::ObstaclePtr obstacle) {
        float t, o;
        road->firstOrthogonalDistance(obstacle->position(), o, t);
        return o;

    }
*/ // XXX End of Legacy Code

} // end namespace trajectory_generator


