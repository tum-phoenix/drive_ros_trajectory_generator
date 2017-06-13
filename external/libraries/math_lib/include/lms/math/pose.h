#pragma once

#include <Eigen/Eigen>
#include <lms/math/interpolation.h>
#include <iostream>
#include <lms/serializable.h>
#include <street_environment/bounding_box.h>
#include <cereal/types/vector.hpp>

namespace lms{
namespace math{
struct Pose2D: public lms::Serializable{
    Pose2D():x(0),y(0),phi(0){
    }
    Pose2D(float x,float y,float phi):x(x),y(y),phi(phi),timeStamp(0){}
    float x;
    float y;
    float phi;
    double timeStamp;

    /////////////////////////////// Serialization //////////////////////////////
    CEREAL_SERIALIZATION()

    template <class Archive>
    void serialize(Archive& archive) {
        archive(x,y,phi,timeStamp);
    }
};

struct CoordinateSystem2D: public lms::Serializable{
    float x,y,phi;
    CoordinateSystem2D():x(0),y(0),phi(0){
    }
    CoordinateSystem2D(const Pose2D &p):x(p.x),y(p.y),phi(p.phi){
    }

    Pose2D transformTo(const Pose2D &pose){
        //we have to manually translate it before as combing rotation and translation matrix does a rotation and afterwards the translation
        Eigen::Vector2f v;
        v.x() = x-pose.x;
        v.y() = y-pose.y;

        //create rotation matrix (passive)
        Eigen::Matrix2f rotMat;
        rotMat(0,0) = cos(phi);
        rotMat(0,1) = sin(phi);
        rotMat(1,0) = -sin(phi);
        rotMat(1,1) = cos(phi);
        v = rotMat*v;

        Pose2D res;
        res.x = v.x();
        res.y = v.y();
        res.phi = phi-pose.phi;
        res.timeStamp = pose.timeStamp;
        return res;
    }
    /////////////////////////////// Serialization //////////////////////////////
    CEREAL_SERIALIZATION()

    template <class Archive>
    void serialize(Archive& archive) {
        archive(x,y,phi);
    }
};
class Pose2DHistory : public lms::Serializable{
    Pose2D m_currentPose;
    std::vector<Pose2D> m_poses;
public:
    std::vector<Pose2D> poses(){
        return m_poses;
    }

    Pose2DHistory():posesMaxSize(-1){

    }
    int posesMaxSize;

    /**
     * @brief update
     * @param dx
     * @param dy
     * @param dphi drehung des Koordinatensystems
     * @param time (can be abused as id) in ms
     */
    void addPose(float x, float y, float phi, double timeStamp){
        m_currentPose.x = x;
        m_currentPose.y = y;
        m_currentPose.phi = phi;
        m_currentPose.timeStamp = timeStamp;
        m_poses.push_back(m_currentPose);
        if(posesMaxSize > 0 && (int)m_poses.size() > posesMaxSize){
            m_poses.erase(m_poses.begin()+1);
            //TODO remove some points in between
        }
    }
    /**
     * @brief getPose
     * @param time
     * @param pose
     * @return false if the time is before the pose or if the time is after the last pose
     */
    bool getPose(const double time,Pose2D &pose) const{
        if(m_poses.size() == 0 || time < m_poses[0].timeStamp)
            return false;
        for(std::size_t i = 1; i < m_poses.size(); i++){
            if(m_poses[i].timeStamp >= time){
                //long should fit into double
                pose.x = lms::math::linearInterpolation<double>(m_poses[i].timeStamp,m_poses[i].x,m_poses[i-1].timeStamp,m_poses[i-1].x,time);
                pose.y = lms::math::linearInterpolation<double>(m_poses[i].timeStamp,m_poses[i].y,m_poses[i-1].timeStamp,m_poses[i-1].y,time);
                pose.phi = lms::math::linearInterpolation<double>(m_poses[i].timeStamp,m_poses[i].phi,m_poses[i-1].timeStamp,m_poses[i-1].phi,time);
                pose.timeStamp = time;
                if(std::isnan(pose.x) || std::isnan(pose.y)  || std::isnan(pose.phi) ){
                    std::cerr<<"pose is nan"<<pose.x << " "<<pose.y<<" "<<pose.phi;
                    return false;
                }
                return true;
            }
        }
        return false;
    }

    Pose2D currentPose() const{
        return m_currentPose;
    }
    /**
     * @brief deltaPose
     * @param time1
     * @param time2
     * @param pose
     * @return
     */
    bool deltaPose(const long time1, const long time2, Pose2D &pose){
        Pose2D pose1,pose2;
        if(!getPose(time1,pose1))
            return false;
        if(!getPose(time2,pose2))
            return false;
        pose.x = pose2.x -pose1.x;
        pose.y = pose2.y -pose1.y;
        pose.phi = pose2.phi -pose1.phi;
        pose.timeStamp = pose2.timeStamp -pose1.timeStamp;
        return true;
    }
    /**
     * @brief deltaPose
     * @param time
     * @param pose
     * @return
     */
    bool deltaPose(const long time, Pose2D &pose){
        Pose2D pose1;
        if(!getPose(time,pose1))
            return false;
        pose.x = currentPose().x -pose1.x;
        pose.y = currentPose().y -pose1.y;
        pose.phi = currentPose().phi -pose1.phi;
        pose.timeStamp = currentPose().timeStamp -pose1.timeStamp;
        return true;
    }

    /////////////////////////////// Serialization //////////////////////////////
    CEREAL_SERIALIZATION()

    template <class Archive>
    void serialize(Archive& archive) {
        archive(m_currentPose,m_poses);
    }
};

}
}
