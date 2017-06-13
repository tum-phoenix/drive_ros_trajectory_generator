#ifndef LMS_MATH_EIGEN_H
#define LMS_MATH_EIGEN_H
#include <Eigen/Dense>

#include "lms/math/vertex.h"
namespace lms {
namespace math {

inline lms::math::vertex2f fromEigenf(const Eigen::Vector2f &v){
    return lms::math::vertex2f(v(0),v(1));
}
inline lms::math::vertex2d fromEigend(const Eigen::Vector2d &v){
    return lms::math::vertex2d(v(0),v(1));
}

template<int rows, int cols>
using Matrix = Eigen::Matrix<float, rows, cols>;

template<int rows>
using Vector = Matrix<rows, 1>;
/**
 * @brief important: p1, p2, p3 must have order
 */
inline float circleCurvature(lms::math::vertex2f p1, lms::math::vertex2f p2, lms::math::vertex2f p3){
    // look at Arndt Brunner for explanation: http://www.arndt-bruenner.de/mathe/scripts/kreis3p.htm


    float kappa_est = 0;

    //set up A matrix
    Matrix<3,3> A;
    A << 1, -p1.x, -p1.y,
        1, -p2.x, -p2.y,
        1, -p3.x, -p3.y;

    Vector<3> b;
    b <<    -(pow(p1.x, 2) + pow(p1.y,2)),
            -(pow(p2.x, 2) + pow(p2.y,2)),
            -(pow(p3.x, 2) + pow(p3.y,2));

    Vector<3> sol;

    sol = A.colPivHouseholderQr().solve(b);

    float xm = sol(1)/2;
    float ym = sol(2)/2;
    float r = sqrt(pow(xm,2) + pow(ym,2) - sol(0));

    if (r == 0)
    {
        //throw some kind of error here
        kappa_est = 0;
    }else
    {
        lms::math::vertex2f v1 = p2 - p1; // from first to second point
        lms::math::vertex2f v2; // from first to middle
        v2.x = xm - p1.x;
        v2.y = ym - p1.y;

        // generate sign
        float thirdComponentCrossProduct = v1.x*v2.y - v1.y*v2.x;

        if (thirdComponentCrossProduct > 0)
        {
            // middle of circle is on the left --> positive curvature
            kappa_est = 1/r;
        } else
        {
            // middle of circle is on the right --> negative curvature
            kappa_est = - 1/r;
        }


    }

    return kappa_est;

}
}//namespace math
}//namespace lms
#endif //LMS_MATH_EIGEN_H
