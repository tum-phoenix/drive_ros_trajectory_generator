//
// Created by Lukas Koestler on 23.10.15.
// Class to represent a Polynomial
//

#ifndef PROJECT_POLY_H
#define PROJECT_POLY_H

#include <lms/logging/logger.h>

#include "types.h"


template <size_t n>

class Poly {




public:
    Poly(const Vector<n+1>& _coeff) : coeff(_coeff)
    {

    }


/**
 * evaluates the polynomial at the points in x
 */
    template<size_t m>
    Vector<m> eval(const Vector<m>& x)
    {

        Vector<m> y;

        double value;
        double x_power;

        for (int i = 0; i <= m-1; i++)
        {
            value = coeff(0);
            x_power = 1.0;

            for (int j = 1; j <= n; j++)
            {
                x_power = x_power*x(i);

                value = value + coeff(j)*x_power;
            }

            y(i) = value;

        }

        return y;
    }

    /**
     * Evaluates the Polynomial at one point
     */
    float evalAtPoint(const float x)
    {
        float value = coeff(0);
        float x_power = 1.0;

        for (int j = 1; j <= n; j++)
        {
            x_power = x_power*x;

            value = value + coeff(j)*x_power;
        }

        return value;
    }

    /**
     * integrates the polynomial
     */
    Poly<n+1> integrate()
    {
        Vector<n+2> new_coeff;

        for (int i = 1; i <= n+1; i++)
        {
            new_coeff(i) = coeff(i-1)/i;
        }

        return Poly<n+1>(new_coeff);
    }

    /**
     * Differentiates the Polynomial
     */
    Poly<n-1> differentiate()
    {
        Vector<n> new_coeff;

        for (int i = 0; i <= n-1; i++)
        {
            new_coeff(i) = (i+1)*coeff(i+1);
        }

        return Poly<n-1>(new_coeff);
    }


    /**
     * @brief gives the coefficients
     */
    const Vector<n+1>& getCoeff() const {
        return coeff;
    }

    friend std::ostream& operator<<(std::ostream& os, const Poly<n>& poly)
    {
        os << "polynomial of degree " << n << " with the coefficients " << poly.getCoeff().transpose();
        return os;
    }

private:
    Vector<n+1> coeff;
    int bla;

};


#endif //PROJECT_POLY_H
