//
// Created by Lukas Koestler on 29.11.15.
//

#ifndef PROJECT_BEZIERPOLYNOMIAL_H
#define PROJECT_BEZIERPOLYNOMIAL_H

#include <cstddef>
#include "types.h"

/**
 * @brief A class to represent Bezier Polynomials in 1-D. Is used by the Bezier Curve class
 */
template <size_t n>
class BezierPolynomial {

public:



    /**
     * @brief constructor give points as Eigen Vector<n+1> t_begin, t_end
     */
    BezierPolynomial(const Vector<n+1> controlPointsIn, const float t_beginIn, const float t_endIn) : t_begin(initBegin(t_beginIn, t_endIn)),
                                                                                     t_end(initEnd(t_beginIn, t_endIn)),
                                                                                     controlPoints(controlPointsIn)
    {

    }

    /**
     * @brief evaluate Polynomial at one point t in [t_begin, t_end]
     */
    float evalAtPoint(const float t)
    {
        if (t < t_begin || t > t_end)
        {
            // throw error
            return 0;
        }
        // maps from t in [t_begin, t_end] to tau in [0, 1]
        float tau = (t-t_begin)/(t_end-t_begin);

        //This is the Casteljaus Algorithm as on wikipedia: https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm
        Vector<n+1> betas_old;
        Vector<n+1> betas_new;

        size_t j = 0;
        // fill with control points: eigen does this
        betas_old = controlPoints;

        for (j = 1; j <= n; j++)
        {
            for (size_t i = 0; i <= n-j; i++)
            {
                betas_new(i) = betas_old(i)*(1-tau)  +   betas_old(i+1)*tau; //formula from wikipedia
            }

            //change beta_old and new
            betas_old = betas_new;
        }

        return betas_new(0);

    }

    /**
     * @brief evaluate Polynomial at m points tt all in [t_begin, t_end]
     */
    template<size_t m>
    Vector<m> eval(const Vector<m> tt)
    {
        Vector<m> result;

        for(size_t i = 0; i < m; i++)
        {
            result(i) = this->evalAtPoint(tt(i));
        }

        return result;
    }

    /**
     * @brief returns the derivative, which again is a BezierPolynomial
     */
    BezierPolynomial<n-1> differentiate()
    {

        //calculate the new control points (see formula on wikipedia)
        Vector<n-1+1> controlPoints_new;

        for (size_t i = 0; i <= n-1; i++)
        {
            controlPoints_new(i) = n*(controlPoints(i+1) - controlPoints(i))  * 1/(t_end-t_begin); //at the end chain rule!!
        }

        // t_begin and t_end don't change
        return BezierPolynomial<n-1>(controlPoints_new, t_begin, t_end);
    }


private:
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

    const Vector<n+1> controlPoints;

    const float t_begin;
    const float t_end;

};


#endif //PROJECT_BEZIERPOLYNOMIAL_H
