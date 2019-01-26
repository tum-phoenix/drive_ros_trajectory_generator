This is the repo for trajectory\_generation.

Inputs (over ROS):
*Format: Topic* | *Datatype*\
ENVIRONMENT\_OBSTACLE | street\_environment::EnvironmentObjects\
ROAD\_STATES | street\_environment::RoadStates\
ROAD | street\_environment::RoadLane\
CAR |  street\_environment::CarCommand

Outputs (over ROS):\
*Format: Topic* | *Datatype*\
LINE | street\_environment::Trajectory\
DEBUG\_TRAJECTORY |  lms::math::polyLine2f


Values that messagetypes have to hold:\
TODO describe how enums are put in msgs\
**EnvironmentObject(s?).msg** (as found in lms/street\_environment/street\_environment.h)\
std::string m_name\
float m_trust\
float m_trustLast\
std::set<std::string> m_detectedBySensorId\
(maybe) int type

**RoadStates.msg** (as found in lms/street\_environment/road.h)\
std::vector<RoadState> states\
    *-> RoadState hierbei:*\
    *RoadStateType type*\
    *float startDistance*\
    *float endDistance*\
    *float probability*\
    *float curvature*\
       *-> RoadStateType hierbei ein enum mit folgenden Werten:*\
       *UNKNOWN, STRAIGHT, STRAIGHT_CURVE, CURVE*

**RoadLane.msg** (as found in lms/street\_environment/road.h)\
RoadLaneType m_type\
(what is: static constexpr int TYPE = 0)?\
    *-> RoadLaneType hierbei ein enum mit folgenden Werten:*\
    *LEFT, MIDDLE, RIGHT*

**CarCommand.msg** (as found in lms/street\_environment/car.h)\
std::vector<State> states\
    *-> State hierbei ein struct mit:*\
    *int priority;*\
    std::string name;\
    StateType state;\
    bool indicatorLeft;\
    bool indicatorRight;\
    lms::Time startState; -> ros::time\
    lms::Time endState; -> ros::time\
    float steering\_front, steering\_rear;\
    float targetSpeed;\
    float targetDistance;\
       *-> StateType hierbei ein enum mit folgenden Werten:*\
       *NOT_DEFINED,IDLE,DRIVING,PARKING,RACE*

**Trajectory.msg** (as found in lms/street\_environment/)\
std::vector<TrajectoryPoint> points\
    *-> TrajectoryPoint hierbei ein struct mit:*\
    lms::math::vertex2f position;\
    lms::math::vertex2f directory;\
    float velocity;\
    float distanceToMiddleLane;\
       *-> lms::math::vertex2f hierbei:*\
       float x\
       float y

**polyLine2f.msg** (as found in lms::math::polyLine2f)\
std::vector\<lms::math::vertex2f> m_points\
    *-> lms::math::vertex2f hierbei:*\
    float x\
    float y
	