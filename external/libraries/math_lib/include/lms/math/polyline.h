#ifndef LMS_MATH_POLYLINE_H
#define LMS_MATH_POLYLINE_H
#include <vector>
#include "lms/math/vertex.h"
#include <algorithm>
#include <functional>
#include <lms/inheritance.h>

#include "lms/serializable.h"
#include "cereal/cerealizable.h"
#include "cereal/cereal.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/base_class.hpp"

namespace lms{
namespace math{

template<typename VERTEX> class PolyLine: public lms::Serializable
{
protected:
    std::vector<VERTEX> m_points;
public:
    virtual ~PolyLine() {}

    const std::vector<VERTEX>& points() const{
        return m_points;
    }
    std::vector<VERTEX>& points(){
        return m_points;
    }

    void move(VERTEX toAdd){
        for(int i = 0; i < m_points.size(); i++){
            points()[i] += toAdd;
        }
    }

    float length(){
        float sum = 0;
        for(int i = 1; i < m_points.size(); i++){
            sum += m_points[i-1].distance(m_points[i]);
        }
        return sum;
    }



    void sort(std::function<bool(const VERTEX&,const VERTEX&)> sortF){
        std::sort(points().begin(), points().end(),sortF);
    }
    /**
     * @brief sortByDistance
     * @param absolue if set to true, the line will be sorted using the distance. Shortest distance will be at beginning
     * if set to false, the line will be sorted using the relative distance to the points. For example if it's a unsorted path of a line and the first point is at the beginning you will receive the line
     */
    void sortByDistance(bool absolute){
        //TODO absolute is easy
        //TODO relative may be quite hard
    }
    /**
     * @brief flip first index will be last and vise versa
     */
    void flip(){
        //TODO
    }

    PolyLine<VERTEX>& operator += (const PolyLine<VERTEX> toAdd){
        for(VERTEX v:toAdd.points()){
            this->points().push_back(v);
        }
        return *this;
    }

    PolyLine<VERTEX> operator -= (const PolyLine<VERTEX> sub){
        //TODO remote vectors that are the same
        return this;
    }

    /**
     * @brief reduce used to reduce unnecessary points. The list should be sorted before!
     * @param distanceF TODO
     */
    void reduce(std::function<bool(const VERTEX&,const VERTEX&)> distanceF){
        for(int i = 0; i < ((int)points().size()) -1;){
            //quite dirty, wouldn't work if the vertex doesn't have a distance method...
            if(distanceF(points()[i],points()[i+1])){
                points().erase(points().begin() + i+1);
            }else{
                i++;
            }
        }
    }
    /**
     * @brief reduce used to reduce unnecessary points.
     * @param distanceF returning true will remove the VERTEX
     */
    void reduce(std::function<bool(const VERTEX&)> distanceF){
        for(int i = 0; i < ((int)points().size()) -2;){
            //quite dirty, wouldn't work if the vertex doesn't have a distance method...
            if(distanceF(points()[i])){
                points().erase(points().begin() + i);
            }else{
                i++;
            }
        }
    }
    /**
     * @brief reduce used to reduce unnecessary points. The list should be sorted before!
     * @param distanceF returning true will remove the second VERTEX
     */
    void reduce(std::function<bool(const VERTEX&,const VERTEX&,const VERTEX&)> distanceF){
        for(int i = 0; i < ((int)points().size()) -2;){
            //quite dirty, wouldn't work if the vertex doesn't have a distance method...
            if(distanceF(points()[i],points()[i+1],points()[i+2])){
                points().erase(points().begin() + i+1);
            }else{
                i++;
            }
        }
    }

    // cereal implementation
        //get default interface for datamanager
        CEREAL_SERIALIZATION()

        template <class Archive>
        void serialize( Archive & archive) {
            archive(m_points);
        }
};



class polyLine2f : public PolyLine<lms::math::vertex2f>, public virtual lms::Inheritance{
public:
    virtual bool isSubType(std::type_index tIndex) const override{
        (void)tIndex;
        return false;
    }
    virtual ~polyLine2f(){}

    float length() const{
        float tmp = 0;
        for(int i = 1; i < (int)m_points.size(); i++){
            tmp += m_points[i-1].distance(m_points[i]);
        }
        return tmp;
    }

    /**
     * @brief startAt
     * @param distance
     * @return a polyLine2f starting at the given distance from the first point along the line
     */
    polyLine2f startAt(float distance) const{
        polyLine2f result;
        float currentDistance = 0;
        for(int i = 1; i <(int) points().size(); i++){
            currentDistance += points()[i].distance(points()[i-1]);
            if(currentDistance > distance){
                //add start point
                result.points().push_back(interpolateAtDistance(distance));
                for(int k = i; k < (int)points().size(); k++){
                    result.points().push_back(points()[k]);
                }
                break;
            }
        }
        return result;
    }

    /**
     * @brief stopAt
     * @param distance
     * @return a polyLine2f stopping at the given distance from the first point along the line
     */
    polyLine2f stopAt(float distance) const{
        polyLine2f result;
        if(points().size() == 0)
            return result;
        float currentDistance = 0;
        result.points().push_back(points()[0]);
        for(int i = 1; i <(int) points().size(); i++){
            currentDistance += points()[i].distance(points()[i-1]);
            if(currentDistance < distance){
                result.points().push_back(points()[i]);
            }else{
                //add the endpoint
                result.points().push_back(interpolateAtDistance(distance));
                break;
            }
        }
        return result;
    }

    /**
     * @brief Returns the shortest distance from the line to v.
     * Line needs to have at least 1 point to get a sensible result. If the line
     * has no points returns 0.
     */
    float shortestDistance(const vertex2f &v) const {
        if (points().size() == 0) {
            return 0;
        }
        float shortest_distance = points().at(0).distance(v);
        for (std::vector<float>::size_type i = 0; i < points().size() - 1; i++) {
            float distance =
                minimum_distance(points().at(i), points().at(i + 1), v);
            shortest_distance = std::min(shortest_distance, distance);
        }
        return shortest_distance;
    }

    /**
     * @brief distance does not give the closest distance, just the first orth distance with sign!
     * @param v
     * @return
     */
    void firstOrthogonalDistance(const lms::math::vertex2f &v, float &orth, float &tang) const{
        orth = 0;
        tang = 0;
        //error handling? size == 0?
        if(m_points.size() == 1){
            tang = (m_points[0]-v).length();
        }
        for(int i = 1; i < (int) m_points.size(); i++){
            float part;
            orth = minimum_distance(m_points[i-1],m_points[i],v,part,false);
            if(lms::math::vertex2f::side(m_points[i-1],m_points[i],v) < 0){
                orth = -orth;
            }

            if(part > 1 && i != ((int)m_points.size()) -1){
                tang += m_points[i-1].distance(m_points[i]);
            }else{
                tang += m_points[i-1].distance(m_points[i]) * part;
                return;
            }
        }
    }

    /**
     *
     * @brief interpolateAtDistance
     * @param distanceIn
     * @return the point with the given distance from the first point along the line
     */
    lms::math::vertex2f interpolateAtDistance(float distanceIn) const{
        lms::math::vertex2f result;

        int nPointsRoad = points().size();
        float lengthEnvModelSegment = points()[0].distance(points()[1]); //TODO für jedes teilstück festlegen?

        float distanceInClean = distanceIn;

        //check if environment model has needed range
        float maxDistanceEnvModel =  lengthEnvModelSegment * (nPointsRoad-1);
        if (distanceInClean >= maxDistanceEnvModel)
        {
           // logger.warn("distanceIn is bigger than max distance of environment model");
            distanceInClean = maxDistanceEnvModel;
            result = points()[nPointsRoad-1];
            return result;

        }
        if (distanceInClean < 0)
        {
           // logger.warn("distanceIn smaller 0");
            distanceInClean = 0;
        }

        // get the point
        if (fmod(distanceInClean, lengthEnvModelSegment) == 0)
        {
            // by chance got one point
            int idPoint = round(distanceInClean/lengthEnvModelSegment);
            result = points()[idPoint];
    //        logger.warn("perfect hit: i point:  ") << idPoint << ",  distance in: " << distanceInClean;
            return result;
        }

        // is between two points
    //    logger.warn("distanceIn   ") << distanceInClean;
        int idPointBefore = floor(distanceInClean/lengthEnvModelSegment);

        if ((idPointBefore < 0) || (idPointBefore > nPointsRoad - 2))
        {

            if (idPointBefore < 0)
            {
              //  logger.warn("the id of the point selected is smaller 0");
                idPointBefore = 0;
            }
            if (idPointBefore > nPointsRoad - 2)
            {
                //logger.warn("the id of the point selected is to big: ") << idPointBefore;
                //logger.warn("nPointsRoad: ") << nPointsRoad;
                idPointBefore = nPointsRoad - 2;
            }
        }

        lms::math::vertex2f pointBefore = points()[idPointBefore];
        lms::math::vertex2f pointAfter = points()[idPointBefore+1]; // not going out of bounds should be automatically detected before


        float fractionFirst =1 - (distanceInClean - idPointBefore*lengthEnvModelSegment)/lengthEnvModelSegment;

        if ((fractionFirst < 0))
        {
        //    logger.warn("fraction should be bigger 0");
            fractionFirst = 0;
        }
        if ((fractionFirst > 1))
        {
        //    logger.warn("fraction should be smaller 1");
            fractionFirst = 1;
        }



        result.x = fractionFirst*pointBefore.x + (1-fractionFirst)*pointAfter.x;
        result.y = fractionFirst*pointBefore.y + (1-fractionFirst)*pointAfter.y;

        //logger.warn("i: ") << idPointBefore << ",  distance in: " << distanceInClean << ",  fraction first: " << fractionFirst <<",  x: " << result.x <<",  y= " << result.y << ", point before: x:" << pointBefore.x << ", y:" << pointBefore.y <<",  point0:x " << points()[0].x << ", y:" << points()[0].y;

        return result;
    }

    /**
     *
     * @brief interpolateTangentAtDistance
     * @param distanceIn
     * @return the tangent at the given distance from the first point along the line
     */
    lms::math::vertex2f interpolateTangentAtDistance(float distanceIn) const{
        lms::math::vertex2f result;
        result.x = 1;
        result.y = 0;

        int nPointsRoad = points().size();
        float lengthEnvModelSegment = points()[0].distance(points()[1]); //TODO für jedes teilstück festlegen?

        float distanceInClean = distanceIn;

        //check if environment model has needed range
        float maxDistanceEnvModel =  lengthEnvModelSegment * (nPointsRoad-1);
        if (distanceInClean >= maxDistanceEnvModel)
        {
          //  logger.warn("distanceIn is bigger than max distance of environment model");
            distanceInClean = maxDistanceEnvModel;
            result = points()[nPointsRoad-1] - points()[nPointsRoad-2];
            return result.normalize();

        }
        if (distanceInClean < 0)
        {
         //   logger.warn("distanceIn smaller 0");
            distanceInClean = 0;
        }

        // get the point
        if (fmod(distanceInClean, lengthEnvModelSegment) == 0)
        {
            // by chance got one point
            int idPoint = round(distanceInClean/lengthEnvModelSegment);
            result = points()[idPoint+1] - points()[idPoint-1];
            return result.normalize();
        }

        int idPointBefore = floor(distanceInClean/lengthEnvModelSegment);

        if ((idPointBefore < 0) || (idPointBefore > nPointsRoad - 2))
        {

            if (idPointBefore < 0)
            {
            //    logger.warn("the id of the point selected is smaller 0");
                idPointBefore = 0;
            }
            if (idPointBefore > nPointsRoad - 2)
            {
            //    logger.warn("the id of the point selected is to big: ") << idPointBefore;
              //  logger.warn("nPointsRoad: ") << nPointsRoad;
                idPointBefore = nPointsRoad - 2;
            }
        }

        lms::math::vertex2f pointBefore = points()[idPointBefore];
        lms::math::vertex2f pointAfter = points()[idPointBefore+1]; // not going out of bounds should be automatically detected before

        result = pointAfter - pointBefore;

        return result.normalize();
    }
    /**
     * @brief interpolateNormalAtDistance
     * @param distanceIn
     * @return the normal at the given distance from the first point along the line
     */
    lms::math::vertex2f interpolateNormalAtDistance(float distanceIn) const{
        lms::math::vertex2f tangent = interpolateTangentAtDistance(distanceIn);
        lms::math::vertex2f normal;
        normal.x = - tangent.y;
        normal.y = tangent.x;

        return normal.normalize();
    }

    /**
     * @brief getWithDistanceBetweenPoints
     * @param distance
     * @return
     */
    polyLine2f getWithDistanceBetweenPoints(const float distance)const {
        polyLine2f result;
        if(distance <= 0){
            LMS_EXCEPTION("invalid distance given: " + std::to_string(distance));
        }
        if(m_points.size() == 0){
            return result;
        }
        int currentIndex = 1;
        result.points().push_back(m_points[0]);//add first point
        while(true){
            lms::math::vertex2f lastPoint = result.points()[result.points().size()-1];
            lms::math::vertex2f next = m_points[currentIndex];
            //find valid next
            //we increase the point number until we found a point that is further away from the last point then the distance
            while(lastPoint.distance(next) < distance){
                currentIndex++;
                if(currentIndex >= (int)m_points.size()){
                    break; //end first loop
                }
                next = m_points[currentIndex];
            }
            //std::cout<<"numerOfNewPoints: "<<result.points().size()<<" currentIndex "<< currentIndex<<std::endl;
            //std::cout<<"next point: "<<next.x << " "<<next.y<<std::endl;
            //std::cout<<"lastPoint point: "<<lastPoint.x << " "<<lastPoint.y<<std::endl;
            //found valid next
            //add new point
            lms::math::vertex2f diff = next-lastPoint;
            if(diff.lengthSquared() != 0){
                result.points().push_back(lastPoint+diff.normalize()*distance);
            }else{
                currentIndex++;
            }
            //Not nice but ok
            if(currentIndex >= (int)m_points.size()){
                break; //end bigger loop
            }
        }
        return result;
    }
    /**
     * @brief moveOrthogonal
     * "distance" is the distance from the orthogonally moved line to
     * the given. It is not the distance moved. A positive distance gives a line
     * that is moved orthogonally in negative direction from the given.
     * @param distance
     * @return empty polyLine2f if the transition is not possible
     */
    polyLine2f moveOrthogonal(float distance) const{
        polyLine2f result;
        //check if it's possible
        if(m_points.size() < 2){
            return result;
        }
        lms::math::vertex2f diff;
        for(int i = 1; i <(int) m_points.size(); i++){
            lms::math::vertex2f top = m_points[i];
            lms::math::vertex2f bot = m_points[i-1];
            diff = top-bot;
            float diffDistance = diff.length();
            if(diffDistance == 0){
                //TODO this could be solved nicely
                //TODO LMS_EXCEPTION("Points should not be the same, diffDistance == 0");
                continue;
            }
            diff = diff.rotateClockwise90deg()*distance/diffDistance;
            result.points().push_back(bot+diff);
        }
        result.points().push_back(m_points.back()+diff);
        return result;
    }

    template <class Archive>
    void serialize( Archive & archive) {
        archive(cereal::base_class<PolyLine<lms::math::vertex2f>>(this));
    }
};
}  // namespace math
}  // namespace lms

#endif /* LMS_MATH_POLYLINE_H */
