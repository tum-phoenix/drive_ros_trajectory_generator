//
// Created by Lukas Koestler on 29.11.15.
//

#ifndef PROJECT_BEZIERCURVE_H
#define PROJECT_BEZIERCURVE_H

#include <vector>
#include <math.h>

#include "BezierPolynomial.h"
/**
 * @brief Class which handles 2d Bezier Curves
 */
template<size_t n>
class BezierCurve {
    /**
     * @brief constructor
     */
public:
    BezierCurve(const points2d<n+1> controlPointsIn, const float t_beginIn, const float t_endIn) : t_begin(initBegin(t_beginIn, t_endIn)),
                                                                                           t_end(initEnd(t_beginIn, t_endIn)),
                                                                                           poly_x(BezierPolynomial<n>(controlPointsIn.x, t_begin, t_end)),//Watch OUT!!!!! This seems to make the mistake as t_beginIn and t_endIn are anythind (mostly 0)
                                                                                           poly_y(BezierPolynomial<n>(controlPointsIn.y, t_begin, t_end)),
                                                                                           poly_dx(poly_x.differentiate()),
                                                                                           poly_dy(poly_y.differentiate()),
                                                                                           poly_ddx(poly_dx.differentiate()),
                                                                                           poly_ddy(poly_dy.differentiate())
    {


    }

    /**
     * @brief evaluate curve at one point for the parametrizing parameter t
     */
    Vector<2> evalAtPoint(const float t)
    {
        Vector<2> result;
        result(0) = 0;
        result(1) = 1;

        if (t < t_begin || t > t_end)
        {
            // throw error
            return result;
        }

        result(0) = poly_x.evalAtPoint(t);
        result(1) = poly_y.evalAtPoint(t);

        return result;

    }

    /**
     * @brief evaluate curve at multiple points for the parametrizing parameter t
     */
    template <size_t m>
    points2d<m> eval(const Vector<m> tt)
    {
        points2d<m> points;
        points.x = poly_x.template eval<m>(tt);
        points.y = poly_y.template eval<m>(tt);


        return points;
    }


    /**
     * @brief return the normal vector at one point
     */
    Vector<2> normalAtPoint(const float t) {
        Vector<2> result;
        result(0) = 0;
        result(1) = 1;

        if (t < t_begin || t > t_end) {
            // throw error
            return result;
        }

        float dy = poly_dy.evalAtPoint(t);
        float dx = poly_dx.evalAtPoint(t);

        float norm = sqrt(pow(dy, 2) + pow(dx, 2));

        if (norm == 0)
        {
            result(0) = -dy;
            result(1) = dx;
        }else
        {
            result(0) = -dy/norm;
            result(1) = dx/norm;
        }
        return result;
    }


    /**+
     * @brief: return the tangential vector at one point
     */
    Vector<2> tangentAtPoint(const float t) {
        Vector<2> result;
        result(0) = 0;
        result(1) = 1;

        if (t < t_begin || t > t_end) {
            // throw error
            return result;
        }

        float dy = poly_dy.evalAtPoint(t);
        float dx = poly_dx.evalAtPoint(t);

        float norm = sqrt(pow(dy, 2) + pow(dx, 2));

        if (norm == 0)
        {
            result(0) = dx;
            result(1) = dy;
        }else
        {
            result(0) = dx/norm;
            result(1) = dy/norm;
        }
        return result;
    }

    float normTangentAtPoint(const float t) {
        float result = 0;

        if (t < t_begin || t > t_end) {
            // throw error
            return result;
        }

        float dy = poly_dy.evalAtPoint(t);
        float dx = poly_dx.evalAtPoint(t);

        result = sqrt(pow(dy, 2) + pow(dx, 2));

        return result;
    }

    template<size_t m>
    points2d<m> tangent(const Vector<m> tt) {
        points2d<m> result;

        Vector<2> result_local;

        for (int j = 0; j< m; j++)
        {
            result_local = this->tangentAtPoint(tt(j));
            result.x(j) = result_local(0);
            result.y(j) = result_local(1);
        }
        return result;
    }


    /**
     * return curvature (signed) at point
     */
    float curvatureAtPoint(const float t)
    {
        float result = 0;

        if (t < t_begin || t > t_end) {
            // throw error
            return result;
        }

        float dy = poly_dy.evalAtPoint(t);
        float dx = poly_dx.evalAtPoint(t);
        float ddx = poly_ddx.evalAtPoint(t);
        float ddy = poly_ddy.evalAtPoint(t);

        float denum = pow( pow(dx,2) + pow(dy,2) , 1.5);
        float num = dx*ddy - dy*ddx;

        if (denum == 0)
        {
            // something wrong
            return result;
        }else
        {
            result = num/denum;
        }
        return result;


    }

    /**
     * return (signed) curvature at multiple points
     */
    template <size_t m>
            Vector<m> curvature(const Vector<m>& tt)
    {
        Vector<m> result;

        for(size_t i = 0; i < m; i++)
        {
            result(i) = this->curvatureAtPoint(tt(i));
        }

        return result;

    }


private:
    const float t_begin;
    const float t_end;

    BezierPolynomial<n> poly_x;
    BezierPolynomial<n> poly_y;

    BezierPolynomial<n-1> poly_dx;
    BezierPolynomial<n-1> poly_dy;

    BezierPolynomial<n-2> poly_ddx;
    BezierPolynomial<n-2> poly_ddy;



    float initBegin(const float t_beginIn, const float t_endIn)
    {
        if (t_beginIn >= t_endIn)
        {
            return 0;
        }else
        {
            return t_beginIn;
        }
    }

    float initEnd(const float t_beginIn, const float t_endIn)
    {
        if (t_beginIn >= t_endIn)
        {
            return 1;
        }else
        {
            return t_endIn;
        }
    }
};


#endif //PROJECT_BEZIERCURVE_H
