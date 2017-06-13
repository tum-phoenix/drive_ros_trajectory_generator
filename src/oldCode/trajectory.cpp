//
// Created by Lukas Koestler on 23.10.15.
//

#include <types.h>
#include "trajectory.h"


/**
 * function that computes if the tra. collides
 */
bool Trajectory::doesCollide() {

    if (collisionDetected == true)
    {
        // if one was already detected, don't do all the comp. again
        return collision;

    }else {

        collisionDetected = true;

        float dt = tend / nSamplesCollisionAndDrivabilityDetection;
        float t = 0;

        float s_local;
        float d_local;

        for (int i = 0; i <= nSamplesCollisionAndDrivabilityDetection; i++) {
            // sample the time numberOfsamplesCollsisionDetection times

            s_local = mPtr_s->evalAtPoint(t); //compute pos. in Frenet space
            d_local = mPtr_d->evalAtPoint(t);


            for (auto const &obs:obstacles) {
                // loop over all obstacles on the road
                if ((s_local >= -safetyS + obs.s0 + obs.v0 * t) && (s_local <= safetyS + obs.s0 + obs.v0 * t)) {
                    // the obstacle is close enough for a collsision and not to far away
                    if (obs.leftLane) {
                        // left lane obstacle
                        if (d_local >= -safetyD) {
                            collision = true;
                            return true;
                        }
                    } else {
                        //right lane obstacle
                        if (d_local <= safetyD) {
                            collision = true;
                            return true;
                        }
                    }
                }
            }

            t = t + dt; //increment time

        }

        // no collision at all
        collision = false;
        return false;
    }
}

/**
 * function that computes if the traj. is physically (no obstacles) drivable
 */
bool Trajectory::isDrivable() {
    if (drivabilityDetected == true)
    {
        // if this was already computed, don't do all the comp. again
        return drivable;
    } else
    {
        drivabilityDetected = true;

        float dt = tend / nSamplesCollisionAndDrivabilityDetection;
        float t = 0;

        // differentiate the polynomials
        Poly<4> poly_d_d = mPtr_d->differentiate(); //first derivative
        Poly<3> poly_d_dd = poly_d_d.differentiate(); //second derivative

        Poly<3> poly_s_d = mPtr_s->differentiate(); //first derivative

        for (int i = 0; i <= nSamplesCollisionAndDrivabilityDetection; i++) {
            // sample in the time nSamplesCollisionAndDrivabilityDetection times
            // evaluate the polynomials for the local time
            float d_d_local  = poly_d_d.evalAtPoint(t);
            float d_dd_local = poly_d_dd.evalAtPoint(t);

            float s_d_local = poly_s_d.evalAtPoint(t);

            // update time
            t = t + dt;

            //compute the local curvature of the traj. in x-y-space
            float kappa_xy_local;

            //catch cases
            if (s_d_local == 0)
            {
                if (d_dd_local == 0)
                {
                    // 0 divided by 0 should be 0 here: so ok
                    kappa_xy_local = 0;
                }else
                {
                    // to not divide by 0
                    drivable = false;
                    return drivable;
                }

            } else
            {
                // normal calculation for kappa_xy_local
                kappa_xy_local = kappa + d_dd_local/s_d_local;

                // Check if the local curvature is bigger than the max. curvature drivable (speed independent)
                if (abs(kappa_xy_local) > kappa_max)
                {
                    drivable = false;
                    return drivable;
                }

                // check if the orthogonal acceleration is ok (speed dependent)
                float v_local_squarred = pow(d_d_local,2) + pow(s_d_local,2);

                if (abs(kappa_xy_local * v_local_squarred) > aOrthMax)
                {
                    drivable = false;
                    return drivable;
                }
            }

        }
        //all drivable
        drivable = true;
        return drivable;
    }
}

/**
 * function that returns the total value of the cost function
 */
float Trajectory::ctot() {
    if (ctot_alreadyCalc) {
        return ctot_value;
    }
    else {

        ctot_alreadyCalc = true;
        //formula from matlab: checked
        ctot_value = coeffCtot1.ks * (coeffCtot1.kj * this->Jts() + coeffCtot1.kT * this->tend) +
                     coeffCtot1.kd * (coeffCtot1.kj * this->Jtd() + coeffCtot1.kT * this->tend);
        return ctot_value;
    }
}

/**
 * function that returns the smoothness-functional for the d direction
 */
float Trajectory::Jtd() {
    const Vector<6> &coeff_d = mPtr_d->getCoeff();
    //formula from matlab: checked
    return 36 * tend * pow(coeff_d(3), 2) + pow(tend, 3) * (192 * pow(coeff_d(4), 2) + 240 * coeff_d(3) * coeff_d(5)) +
           720 * pow(tend, 5) * pow(coeff_d(5), 2) + 144 * pow(tend, 2) * coeff_d(3) * coeff_d(4) +
           720 * pow(tend, 4) * coeff_d(4) * coeff_d(5);
}

/**
 * function that returns the smoothness-functional for the d direction
 */
float Trajectory::Jts() {
    const Vector<5> &coeff_s = mPtr_s->getCoeff();
    //formula from matlab: checked
    return 192 * pow(tend, 3) * pow(coeff_s(4), 2) + 144 * pow(tend, 2) * coeff_s(3) * coeff_s(4) +
           36 * tend * pow(coeff_s(3), 2);
}


/**
 * constructor
 */
Trajectory::Trajectory(const float &_v1, const float &_d1, const float & _safetyS, const float& _safetyD, const RoadData &_roadData1, const std::vector<ObstacleData> &_obstacles, float _tend, const CoeffCtot& _coeffCtot) : roadData1(_roadData1),
                                                                                                                                                                                                           obstacles(_obstacles),
                                                                                                                                                                                                           tend(_tend),
                                                                                                                                                                                                           coeffCtot1(_coeffCtot),
                                                                                                                                                                                                           safetyS(_safetyS),
                                                                                                                                                                                                           safetyD(_safetyD)

{
    //init all variables
    this->kappa = _roadData1.kappa;

    ctot_alreadyCalc = false;

    // convert roadData to initial conditions in Frenet space
    this->S.v0 = cos(roadData1.phi) * roadData1.vx0;
    this->S.a0 = -sin(roadData1.phi) * roadData1.w * roadData1.vx0 + cos(roadData1.phi) * roadData1.ax0;
    this->S.v1 = _v1;

    double cosphi_d = -sin(roadData1.phi) * roadData1.w;
    double cosphi_dd = -pow(roadData1.w, 2) * cos(roadData1.phi);
    double y0_d = sin(roadData1.phi) * roadData1.vx0;
    double y0_dd = -(cos(roadData1.phi) * roadData1.w * roadData1.vx0 + sin(roadData1.phi) * roadData1.ax0);

    this->D.d0 = -cos(roadData1.phi) * roadData1.y0;
    this->D.d0d = -(cosphi_d * roadData1.y0 + cos(roadData1.phi) * y0_d);
    this->D.d0dd = -(cosphi_dd * roadData1.y0 + 2 * cosphi_d * y0_d + cos(roadData1.phi) * y0_dd);
    this->D.d1 = _d1;

    // init polynomial s
    Vector<5> coeff_s;
    // precomputed formulas
    coeff_s << 0, this->S.v0, this->S.a0 / 2, -pow(1 / tend, 2) * (3 * this->S.v0 - 3 * this->S.v1 + 2 * tend * this->S.a0) / 3, pow(1 / tend, 3) *
                                                                                                   (2 * this->S.v0 -
                                                                                                    2 * this->S.v1 +
                                                                                                    tend * this->S.a0) / 4;

    Vector<6> coeff_d;
    // precomputed formulas
    coeff_d << this->D.d0, this->D.d0d, this->D.d0dd / 2, -pow(1 / tend, 3) *
                                        (3 * this->D.d0dd * pow(tend, 2) + 12 * this->D.d0d * tend + 20 * this->D.d0 - 20 * this->D.d1) / 2,
            pow(1 /
                tend, 4) * (3 * this->D.d0dd * pow(
                    tend, 2) + 16 * this->D.d0d * tend + 30 * this->D.d0 - 30 * this->D.d1) / 2, -pow(1 / tend, 5) *
                                                                               (this->D.d0dd * pow(tend, 2) +
                                                                                6 * this->D.d0d * tend + 12 * this->D.d0 -
                                                                                12 * this->D.d1) / 2;

    //Init polynomials
    /*mPtr_s = std::unique_ptr<Poly<4>>(new Poly<4>(coeff_s));
    mPtr_d = std::unique_ptr<Poly<5>>(new Poly<5>(coeff_d));*/

    mPtr_s = new Poly<4>(coeff_s);
    mPtr_d = new Poly<5>(coeff_d);
}
