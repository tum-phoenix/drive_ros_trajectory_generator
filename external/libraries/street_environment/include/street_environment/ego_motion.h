#pragma once

#include <Eigen/Eigen>

template<typename T>
class EgoMotion2D{

    typedef Eigen::Matrix<T,2,1> vec;

    struct Stamp{
        Eigen::Matrix<T,3,3> rotTrans;
        long time;
    };
    Stamp currentRotTrans;

    std::vector<Stamp> stamps;

public:

    vec position(){
        return currentRotTrans.rotTrans.block<2,1>(2,0);
    }
    /**
     * @brief dPosition
     * @return delta position in the frame of the last stamp
     */
    vec dPosition(){
        return stamps[stamps.size()-1].rotTrans.block<2,1>(2,0);
    }
    long dt(){
        return stamps[stamps.size()-1].time-stamps[stamps.size()-2].time;
    }

    /**
     * @brief velocity
     * @return velocity() in the frame of the last stamp
     */
    vec velocity(){
        return dPosition()/dt();
    }

    T phi(){
        return std::atan2(currentRotTrans.rotTrans(1,0),currentRotTrans.rotTrans(0,0));
    }
    T dPhi(){
        return std::atan2(stamps[stamps.size()-1].rotTrans(1,0),stamps[stamps.size()-1].rotTrans(0,0));
    }

    T omega(){
        return dPhi()/dt();
    }

    void addStamp(Stamp s){
        stamps.push_back(s);
        currentRotTrans.rotTrans = currentRotTrans.rotTrans*s.rotTrans;
    }

    void addStamp(T dx, T dy, T dPhi, long time){
        Stamp s;
        s.time = time;
        s.rotTrans(0,0) = std::cos(dPhi);
        s.rotTrans(0,1) = -std::sin(dPhi);
        s.rotTrans(1,0) = std::sin(dPhi);
        s.rotTrans(1,1) = std::cos(dPhi);
        s.rotTrans(0,2) = dx;
        s.rotTrans(1,2) = dy;
        s.rotTrans(2,0) = 0;
        s.rotTrans(2,1) = 0;
        s.rotTrans(2,2) = 1;
        addStamp(s);
    }

    Stamp getStampFromTime(long time){
        if(stamps.size() == 0){
            return currentRotTrans;
        }
        for(std::size_t i = 0; i < stamps.size(); i++){
            if(stamps[i].time < time){
                return stamps[i];
            }
        }
        return stamps[stamps.size()];
    }

    int stampCount(){
        return stamps.size();
    }
};

typedef EgoMotion2D<int> EgoMotion2Di; //if someone uses this, please send me an email saying why :)
typedef EgoMotion2D<float> EgoMotion2Df;
typedef EgoMotion2D<double> EgoMotion2Dd;
