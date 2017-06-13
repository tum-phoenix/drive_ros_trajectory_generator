#ifndef LMS_MATH_INTERPOLATION_H
#define LMS_MATH_INTERPOLATION_H

namespace lms {
namespace math {

/**
 * @brief Compute the linear interpolation of (x0, y0) and
 * (x1, y1) at point x. Returns the result in parameter y.
 *
 * @return true if interpolation was successful, false if x1 - x0 == 0.
 *
 * http://en.wikipedia.org/wiki/Linear_interpolation
 */
template<typename T>
T linearInterpolation(const T x0,const T y0,const T x1,const T y1,const T x) {
    const T dx = x1 - x0;

    if(dx == 0) {
        return y0;
    } else {
        return y0 + (y1 - y0) * (x - x0) / dx;
    }
}

template<typename T>
struct LinearInterpolator{
    T x0;
    T y0;
    T x1;
    T y1;

    T interpolate(const T x){
        return linearInterpolation<T>(x0,y0,x1,y1,x);
    }
};

template<typename T>
struct LinearWeightInterpolator{
    T offSet;
    T maxHeight;
    T x0;
    T x1;
    T xMax;
    T interpolate(const T x){
        T weight = offSet;
        T y = maxHeight-offSet;
        if(x <= xMax){
            weight += y*(1-(xMax-x)/(xMax-x0));//
        }else{
            weight += y*(1-(x-xMax)/(x1-xMax));
        }
        return weight;
    }
};

template<typename T>
struct LinearWeightAverageInterpolator{
    LinearWeightInterpolator<T> inter;
private:
    T m_sum;
    T m_totalWeight;
public:
    void start(){
        m_sum = 0;
        m_totalWeight = 0;
    }
    void add(T toAdd,T x){
        T tmpWeight = inter.interpolate(x);
        m_totalWeight += tmpWeight;
        m_sum+=toAdd*tmpWeight;
    }
    T average(){
        return m_sum/m_totalWeight;
    }
};



}  // namespace math
}  // namespace lms

#endif /* LMS_MATH_INTERPOLATION_H */
