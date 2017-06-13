#ifndef LMS_MATH_VERTEX_H
#define LMS_MATH_VERTEX_H
#include <cmath>
#include <iostream>
#include "math.h"

#include "lms/serializable.h"
#include "cereal/cerealizable.h"
#include "cereal/cereal.hpp"

namespace lms {
namespace math {

/**
 * @brief Implementation of a 2-dimensional vector.
 */
template <typename T>
class vertex2: public lms::Serializable
{
public:
    virtual ~vertex2() {}

    T x, y;

    vertex2() : x(0), y(0) {}

    vertex2(const T &x, const T &y) : x(x), y(y) {}

    vertex2(const vertex2<T> &v) =default;
    vertex2<T>& operator = (const vertex2<T> &v) =default;

    template<typename U>
    explicit vertex2(const vertex2<U> &v) : x(T(v.x)), y(T(v.y)) {}

    T slope() const {
        return y / x;
    }
    template<typename C>
    explicit operator C(){
        return C(x,y);
    }

    vertex2<T> rotate(float angle){
        return vertex2<T>(x * cos(angle) - y * sin(angle),x * sin(angle) + y * cos(angle));
    }

    T slope(const vertex2<T>& v) const {
        return (v.y - y)/(v.x - x);
    }

    float angle() const {
        return atan2(y ,x);
    }

    /**
     * @brief angleBetween
     * @param v
     * @return value between 0 and PI
     */
    float angleBetween(const vertex2<T> &v) const {
        float value = (*this * v) / length() / v.length();
        if(value > 1)
            value = 1;
        if(value < -1)
            value = -1;
        return acos(value);
    }

    bool left(const vertex2<T> &v){
        float orient = x * v.y - y * v.x; //crossproduct
        return orient >0;

    }

    /**
     * @brief angleBetweenWithOrientation
     * @param v
     * @return value between -PI and PI
     */
    float angleBetweenWithOrientation(const vertex2<T> &v) const {
        float orient = x * v.y - y * v.x; //crossproduct
        float angle = angleBetween(v);
        if(orient < 0){
           angle = -angle;
        }
        return angle;
    }

    /**
     * @brief Compute the squared length of the vector.
     */
    T lengthSquared() const {
        return (*this) * (*this);
    }

    /**
     * @brief Compute the length of the vector.
     */
    float length() const {
        return sqrt(lengthSquared());
    }

    /**
     * @brief Compute the squared distance between this vertex and the
     * parameter.
     */
    T distanceSquared(const vertex2<T> &to) const {
        return (*this - to).lengthSquared();
    }

    /**
     * @brief Compute the distance between this vertex and the parameter.
     */
    float distance(const vertex2<T> &to) const {
        return (*this - to).length();
    }

    vertex2<T>& operator += (const vertex2<T> &add) {
        this->x += add.x;
        this->y += add.y;
        return *this;
    }

    vertex2<T>& operator -= (const vertex2<T> &sub) {
        this->x -= sub.x;
        this->y -= sub.y;
        return *this;
    }

    vertex2<T>& operator *= (const T& mul) {
        this->x *= mul;
        this->y *= mul;
        return *this;
    }

    vertex2<T>& operator /= (const T& div) {
        this->x /= div;
        this->y /= div;
        return *this;
    }

    bool operator == (const vertex2<T> &v) const {
        return x == v.x && y == v.y;
    }

    bool operator != (const vertex2<T> &v) const {
        return ! (*this == v);
    }

    vertex2<T> operator + (const vertex2<T> &v) const {
        return vertex2<T>(x + v.x, y + v.y);
    }

    vertex2<T> operator - (const vertex2<T> &v) const {
        return vertex2<T>(x - v.x, y - v.y);
    }

    /**
     * @brief Scale the vector
     */
    vertex2<T> operator * (const T &mul) const {
        return vertex2<T>(x * mul, y * mul);
    }

    /**
     * @brief Scale the vector
     */
    vertex2<T> operator / (const T &div) const {
        return vertex2<T>(x / div, y / div);
    }

    /**
     * @brief Scalar product
     */
    T operator * (const vertex2<T> &v) const {
        return x * v.x + y * v.y;
    }

    /**
     * @brief Normalize the vector to length 1.
     */
    vertex2<T> normalize() const {
        return (*this) / length();
    }

    /**
     * @brief Invert x and y components. The returned vector points to
     * the opposite direction.
     */
    vertex2<T> negate() const {
        return (*this) * (-1);
    }

    vertex2<T> operator - () const {
        return negate();
    }

    vertex2<T> rotateClockwise90deg() const {
        return vertex2<T>(y, -x);
    }

    vertex2<T> rotateAntiClockwise90deg() const {
        return vertex2<T>(-y, x);
    }

    /**
     * @brief side
     * @param v point on the line
     * @param w point on the line
     * @param p the point to check
     * @return  If the formula is equal to 0, the points are colinear. If the line is horizontal, then this returns true if the point is above the line.
     * Taken from: http://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
     */
    static T side(vertex2<T> v,vertex2<T> w, vertex2<T> p){
        return sgn<T>(w.x - v.x)*(p.y - v.y) - (w.y - v.y)*(p.x - v.x);
    }

    /**
     * @brief cross the cross-product in 2D
     * @return
     */
    static T cross(vertex2<T> a,vertex2<T> b){
        return (b.x-a.y)*(a.x-b.y);
    }

    // cereal implementation
    //get default interface for datamanager
    CEREAL_SERIALIZATION()

    //cereal methods
    template<class Archive>
    void serialize(Archive & archive) {
        archive (x, y);
    }
};

template<typename T>
std::ostream& operator<< (std::ostream& os, const vertex2<T> &v) {
    return os << v.x << ", " << v.y;
}

typedef vertex2<int>    vertex2i;
typedef vertex2<float>  vertex2f;
typedef vertex2<double> vertex2d;

/**
 * @brief minimum_distance
 * @param v first point of the line
 * @param w second point of the line
 * @param p
 * @param onTheSegment between 0 for first point, 1 for second point
 * @return
 */
inline float minimum_distance(vertex2f v, vertex2f w, vertex2f p, float &onTheSegment,bool orthOnSeg = true) {
    // Return minimum distance between line segment vw and point p
    const float l2 = v.distanceSquared(w);  // i.e. |w-v|^2 -  avoid a sqrt
    if (l2 == 0.0) return p.distance(v);   // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line.
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    onTheSegment = (p - v)*(w - v) / l2;
    if(orthOnSeg){
        if (onTheSegment < 0.0) return p.distance(v);       // Beyond the 'v' end of the segment
        else if (onTheSegment > 1.0) return p.distance(w);  // Beyond the 'w' end of the segment
    }
    const vertex2f projection = v + (w - v)*onTheSegment;  // Projection falls on the segment
    return p.distance(projection);
}

inline float minimum_distance(vertex2f v, vertex2f w, vertex2f p) {
    float dummy;
    return minimum_distance(v,w,p,dummy);
}

/**
 * @brief sign from
 * http://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
 * @param p1
 * @param p2
 * @param p3
 * @return
 */
inline float sign(const lms::math::vertex2f &p1, const lms::math::vertex2f &p2,
                  const lms::math::vertex2f &p3) {
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}
/**
 * @brief pointInTriangle from
 * http://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
 * It is important that the vertices of the triangle are ordered anticlockwise.
 * Also points that are very close to the edges might slip through due to
 * floating point errors. For examples see the tests.
 *
 * @return true if the point pt is in the area stretched by v1, v2 and v3.
 * This also includes points on the edges of the triangle.
 */
inline bool pointInTriangle(const lms::math::vertex2f &pt,
                            const lms::math::vertex2f &v1,
                            const lms::math::vertex2f &v2,
                            const lms::math::vertex2f &v3) {
    bool b1, b2, b3;

    b1 = sign(pt, v1, v2) < 0.0f;
    b2 = sign(pt, v2, v3) < 0.0f;
    b3 = sign(pt, v3, v1) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
}

}  // namespace math
}  // namespace lms

#endif /*LMS_MATH_VERTEX_H*/
