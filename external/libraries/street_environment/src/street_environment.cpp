#include "street_environment/obstacle.h"
#include "street_environment/road.h"
#include "street_environment/crossing.h"
#include "street_environment/start_line.h"
#include "street_environment/dynamic_entity.h"
#include "street_environment/car.h"
#include "street_environment/ego_motion.h"


#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/polymorphic.hpp"

CEREAL_REGISTER_TYPE(street_environment::CarCommand)
CEREAL_REGISTER_TYPE(street_environment::DynamicEntity)
CEREAL_REGISTER_TYPE(street_environment::RoadLane)
CEREAL_REGISTER_TYPE(street_environment::Obstacle)
CEREAL_REGISTER_TYPE(street_environment::Crossing)
CEREAL_REGISTER_TYPE(street_environment::StartLine)
CEREAL_REGISTER_DYNAMIC_INIT(street_environment)
