#ifndef STREET_ENVIRONMENT_TRAJECTORY_H
#define STREET_ENVIRONMENT_TRAJECTORY_H
#include "lms/math/polyline.h"

#include <cereal/types/base_class.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/access.hpp>
#include <cereal/cerealizable.h>

namespace street_environment {

    struct TrajectoryPoint:public lms::Serializable{
        lms::math::vertex2f position;
        lms::math::vertex2f directory;
        float velocity;
        float distanceToMiddleLane;
        /**
         * @brief isRight
         * @return true if the TrajectoryPoint is on the left side of the middle of the road
         */
        bool isRight() const{
            return distanceToMiddleLane < 0;
        }

        TrajectoryPoint():position(0,0),directory(1,0),velocity(0),distanceToMiddleLane(0){

        }
        TrajectoryPoint(lms::math::vertex2f pos,lms::math::vertex2f viewDir, float velocity_, float distanceToMiddleLane_):position(pos),directory(viewDir),velocity(velocity_),distanceToMiddleLane(distanceToMiddleLane_){

        }
        CEREAL_SERIALIZATION()

        template<class Archive>
        void serialize(Archive &ar) {
            ar(position, directory, velocity,distanceToMiddleLane);
        }
    };

    class Trajectory: public std::vector<TrajectoryPoint>, public lms::Inheritance, public lms::Serializable {
    public:

        Trajectory getWithDistanceBetweenPoints(const float distance)const {
            Trajectory result;
            if(distance <= 0){
                LMS_EXCEPTION("invalid distance given: " + std::to_string(distance));
            }
            if(size() == 0){
                return result;
            }
            int currentIndex = 1;
            result.push_back((*this)[0]);//add first point
            while(true){
                TrajectoryPoint lastPoint = result[result.size()-1];
                TrajectoryPoint next = (*this)[currentIndex];
                //find valid next
                //we increase the point number until we found a point that is further away from the last point then the distance
                while(lastPoint.position.distance(next.position) < distance){
                    currentIndex++;
                    if(currentIndex >= (int)size()){
                        break; //end first loop
                    }
                    next = (*this)[currentIndex];
                }
                //std::cout<<"numerOfNewPoints: "<<result.points().size()<<" currentIndex "<< currentIndex<<std::endl;
                //std::cout<<"next point: "<<next.x << " "<<next.y<<std::endl;
                //std::cout<<"lastPoint point: "<<lastPoint.x << " "<<lastPoint.y<<std::endl;
                //found valid next
                //add new point
                lms::math::vertex2f diff = next.position-lastPoint.position;
                if(diff.lengthSquared() != 0){
                    TrajectoryPoint newT;
                    newT.position = lastPoint.position+diff.normalize()*distance;
                    newT.velocity = (lastPoint.velocity*newT.position.distance(lastPoint.position)+next.velocity*newT.position.distance(next.position))/diff.length();
                    //TODO hack
                    newT.distanceToMiddleLane = next.distanceToMiddleLane;
                    newT.directory = next.directory;
                    result.push_back(newT);
                }else{
                    currentIndex++;
                }
                //Not nice but ok
                if(currentIndex >= (int)size()){
                    break; //end bigger loop
                }
            }
            return result;
        }

        template<class Archive>
        void serialize(Archive &archive) {
            archive(cereal::base_class<std::vector<TrajectoryPoint>>( this ));
        }

        CEREAL_SERIALIZATION()

        virtual bool isSubType(std::type_index tIndex) const override{
            return tIndex == typeid(std::vector<TrajectoryPoint>);
        }
    };
}

CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(street_environment::Trajectory, cereal::specialization::member_serialize )

#endif //STREET_ENVIRONMENT_TRAJECTORY_H

