#ifndef LMS_MATH_LOOKUP_TABLE_H
#define LMS_MATH_LOOKUP_TABLE_H

#include <vector>
#include <lms/math/interpolation.h>
#include <iostream>
#include <stdexcept>

namespace lms {
namespace math {

enum class LookupTableOrder {
    ASC, DESC
};

template<typename T, LookupTableOrder ORDER>
inline bool checkParams(const std::vector<T> &vx, const std::vector<T> &vy, T x) {
    if(vx.empty() || vy.empty() || vx.size() != vy.size()) {
        return false;
    }

    if((ORDER == LookupTableOrder::ASC) && (x < vx[0] || x > vx[vx.size() - 1])) {
        return false;
    }

    if(ORDER == LookupTableOrder::DESC && (x > vx[0] || x < vx[vx.size() - 1])) {
        return false;
    }

    return true;
}

template<typename T, LookupTableOrder ORDER>
bool lookupTableBinarySearch(const std::vector<T> &vx, const std::vector<T> &vy, T x, T &y) {
    if(! checkParams<T,ORDER>(vx, vy, x)) {
        return false;
    }

    int left = 0, right = vx.size() - 1, m;

    while(left <= right) {
        m = (left + right) / 2;

        if(right - left == 1) {
            y = linearInterpolation(vx[left], vy[left], vx[right], vy[right], x);
            return true; //TODO removed weird check
        } else if(right - left == 0) {
            return false;
        }

        if(x == vx[m]) {
            y = vy[m];
            return true;
        } else if(ORDER == LookupTableOrder::ASC ? x < vx[m] : x > vx[m]) {
            right = m;
        } else {
            left = m;
        }
    }

    return false;
}

template<typename T, LookupTableOrder ORDER>
bool lookupTableLinearSearch(const std::vector<T> &vx, const std::vector<T> &vy, T x, T &y) {
    if(! checkParams<T,ORDER>(vx, vy, x)) {
        return false;
    }

    for(int i = 1; i < vx.size(); i++) {
        if(ORDER == LookupTableOrder::ASC ? x <= vx[i] : x >= vx[i]) {
            y= linearInterpolation(vx[i-1], vy[i-1], vx[i], vy[i], x);
        }
    }

    return true;//TODO not sure about the return type...
}

template<typename T, LookupTableOrder ORDER>
struct LookupTable{
    std::vector<T> vx;
    std::vector<T> vy;
    T linearSearch(T x){
        if(vx.size() != vy.size() || vx.size() == 0){
            throw std::runtime_error("lookuptable size invalid: "+std::to_string(vx.size())+ " , "+std::to_string(vy.size()));
        }
        if(toSmall(x))
            return yMin();
        if(toBig(x))
            return yMax();
        T y;
        lookupTableLinearSearch<T,ORDER>(vx,vy,x,y);
        return y;
    }
    T binarySearch(T x){
        if(toSmall(x))
            return yMin();
        if(toBig(x))
            return yMax();
        T y;
        lookupTableLinearSearch<T,ORDER>(vx,vy,x,y);
        return y;
    }

    T yMin(){
        if(ORDER == LookupTableOrder::ASC){
            return vy[0];
        }
        if(ORDER == LookupTableOrder::DESC){
            return vy[vy.size() -1];
        }
    }
    T yMax(){
        if(ORDER == LookupTableOrder::DESC){
            return vy[0];
        }
        if(ORDER == LookupTableOrder::ASC){
            return vy[vy.size() -1];
        }
    }

    bool toSmall(float x){
        return ((ORDER == LookupTableOrder::ASC) && x < vx[0]) ||(ORDER == LookupTableOrder::DESC && x < vx[vx.size() - 1]);
    }
    bool toBig(float x){
        return ((ORDER == LookupTableOrder::ASC) && x > vx[vx.size()-1]) ||(ORDER == LookupTableOrder::DESC && x > vx[0]);
    }
};


}  // namespace math
}  // namespace lms

#endif /* LMS_MATH_LOOKUP_TABLE_H */
