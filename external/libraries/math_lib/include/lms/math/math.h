#ifndef LMS_MATH_MATH_H
#define LMS_MATH_MATH_H
#include <iostream>

namespace lms {
namespace math {
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
/**
 * @brief The Rect struct give left, bot corner
 */
struct Rect{
    float x;
    float y;
    float width;
    float height;
    bool contains(float _x, float _y){
        return _x >= x && _y >= y && _x <= x+width && _y <= y+height;
    }
};

/**
 * @brief limitAngle_0_2PI set the angle in the range from 0 to 2PI
 */
float limitAngle_0_2PI(float angle);
/**
 * @brief limitAngle_nPI_PI set the angle in the range from -PI to PI
 */
float limitAngle_nPI_PI(float angle);

}
}
#endif
