//
// Created by Lukas Koestler on 23.10.15.
//

#ifndef PROJECT_TRAJECTORY_H
#define PROJECT_TRAJECTORY_H

#include "types.h"
#include "poly.h"
#include "BezierCurve.h"
#include <vector>
#include <types.h>
#include <iomanip>
#include <math.h>
#include <cmath>
#include <lms/math/vertex.h>
#include <street_environment/trajectory.h>

/**
 * @brief: holds a trajectory in Frenet space
 */
class Trajectory {
public:
    bool doesCollide(); //does the trajectory collide
    bool isDrivable(); //is the traj. drivable

    float kappa_max = 2.5*4; //max. drivable curvature of the traj.
    float aOrthMax = 0.5*9.81*8; //max. acc. orthogonal to the traj.

    float ctot(); //function: gives back the value of the total cost function

    float tend; //time the traj. needs (as tstart = 0 by definition)

    /**
     * should not be used anymore. Better use projectOntoBezierCurve
     */
    template<size_t m>
    points2d<m> sampleXY()
    {

        points2d<m> points;

        float dt = tend/(m-1); //time steps
        Vector<m> tt;
        for (int i = 0; i < m; i++)
        {
            tt(i) = dt*i;
        }

        Vector<m> ss;
        Vector<m> dd;

        ss = mPtr_s->eval<m>(tt);
        dd = mPtr_d->eval<m>(tt);

        Vector<2> XY;
        Vector<2> vec_help;

        Matrix<2, 2> R; //rot. matrix
        R(0,0) =  cos(this->roadData1.phi);
        R(0,1) = -sin(this->roadData1.phi);
        R(1,0) =  sin(this->roadData1.phi);
        R(1,1) =  cos(this->roadData1.phi);

        // generate the XY points
        if (fabs(this->kappa) > 0)
        {
            for (int i = 0; i <m; i++)
            {
                // Position of the center line in some other XY coordinate system
                XY(0) = sin(this->kappa*ss(i))/this->kappa  - sin(this->kappa*ss(i))*dd(i);
                XY(1) = (1-cos(this->kappa*ss(i)))/this->kappa +  cos(this->kappa*ss(i))*dd(i);

                vec_help = R*XY;

                points.x(i) = vec_help(0) + sin(this->roadData1.phi)*this->D.d0;
                points.y(i) = vec_help(1) - cos(this->roadData1.phi)*this->D.d0;

            }

        } else{
            for (int i = 0; i <m; i++)
            {
                // don't know exactly what this is
                XY(0) = ss(i);
                XY(1) = dd(i);

                vec_help = R*XY;

                //stays the same
                points.x(i) = vec_help(0) + sin(this->roadData1.phi)*this->D.d0;
                points.y(i) = vec_help(1) - cos(this->roadData1.phi)*this->D.d0;

            }


        }

        return points;
    }


    /**
     * Projects the trajectory on the unique Bezier Curve that is defined by the k points in points (points2d<k> struct points)
     * l = diastance between two successive points
     */
    template<size_t m, size_t k>
    points2d<m> projectOntoBezierCurve(const points2d<k> pointsIn, const float l)
    {
        float s_begin = 0;
        float s_end = l*(k-1); //as there are (k-1) segemnts of length l between k points

        BezierCurve<k-1> centerLine = BezierCurve<k-1>(pointsIn, s_begin, s_end);

        //generate the vector with the times
        points2d<m> pointsOut;

        float dt = tend/(m-1); //time steps
        Vector<m> tt;
        for (int i = 0; i < m; i++)
        {
            tt(i) = dt*i;
        }

        Vector<m> ss;
        Vector<m> dd;

        ss = mPtr_s->eval<m>(tt);
        dd = mPtr_d->eval<m>(tt);

        for(size_t i = 0; i < m; i++)
        {
            //different cases
            if (ss(i) < 0)
            {
                // this should not be: throw error
                pointsOut.x(i) = 0;
                pointsOut.y(i) = 0;


            }else if(ss(i) >= 0 && ss(i) <= s_end)
            {
                //point on the center line
                Vector<2> centerLinePoint = centerLine.evalAtPoint(ss(i));

                //normal
                Vector<2> centerLineNormal = centerLine.normalAtPoint(ss(i));

                Vector<2> trajPoint = centerLinePoint + dd(i)*centerLineNormal; //the trajectoray point is the centerLinePoint plus d times the normal (should be oriented the right way and normalized)

                pointsOut.x(i) = trajPoint(0);
                pointsOut.y(i) = trajPoint(1);


            } else if (ss(i) > s_end)
            {
                //the point is behind the trajectory --> linear extrapolation
                //point on the center line
                Vector<2> centerLinePoint = centerLine.evalAtPoint(s_end);

                //normal
                Vector<2> centerLineNormal = centerLine.normalAtPoint(s_end);

                //normal
                Vector<2> centerLineTangent = centerLine.tangentAtPoint(s_end);


                Vector<2> trajPoint = centerLinePoint + (ss(i)-s_end)*centerLineTangent + dd(i)*centerLineNormal;

                pointsOut.x(i) = trajPoint(0);
                pointsOut.y(i) = trajPoint(1);

            } else
            {
                // case not considered: use zeros
                // this should not be: throw error
                pointsOut.x(i) = 0;
                pointsOut.y(i) = 0;
            }
        }


        return pointsOut;

    }

    /**
     * Projects the trajectory on the unique Bezier Curve that is defined by the k points in points (points2d<k> struct points). Also return tangent + velocity
     * l = diastance between two successive points
     */
    template<size_t m, size_t k>
    street_environment::Trajectory projectOntoBezierCurvePlusVelocity(const points2d<k> pointsIn, const float l)
    {

        // initialize
        // generate output
        street_environment::Trajectory trajectoryOut;
        street_environment::TrajectoryPoint toAdd;

        Poly<3> poly_s_d = mPtr_s->differentiate(); //first derivative

        float s_begin = 0;
        float s_end = l*(k-1); //as there are (k-1) segemnts of length l between k points

        BezierCurve<k-1> centerLine = BezierCurve<k-1>(pointsIn, s_begin, s_end);


        float dt = tend/(m-1); //time steps
        Vector<m> tt;
        for (int i = 0; i < m; i++)
        {
            tt(i) = dt*i;
        }

        Vector<m> ss;
        Vector<m> ss_d;
        Vector<m> dd;

        ss = mPtr_s->eval<m>(tt);
        ss_d = poly_s_d.eval<m>(tt);

        dd = mPtr_d->eval<m>(tt);

        float s_end_trajectory = mPtr_s->evalAtPoint(this->tend);

        // get rid of strange values in ss, dd and ss_d i.e. all for which tt(i) > this.T
        for (size_t j = 0; j < m; j++) {
            if (tt(j) > this->tend)
            {
                ss(j) = s_end_trajectory + S.v1*(tt(j) - this->tend);
                dd(j) = D.d1;

                ss_d(j) = S.v1;
            }

        }


        for(size_t i = 0; i < m; i++)
        {
            //different cases
            if (ss(i) < 0)
            {
                // this should not be: throw error
                toAdd.position.x = 0;
                toAdd.position.y = 0;

                toAdd.directory.x = 0;
                toAdd.directory.y = 0;

                toAdd.velocity = 0;

                toAdd.distanceToMiddleLane = 0;


            }else if(ss(i) >= 0 && ss(i) <= s_end)
            {
                Vector<2> centerLinePoint = centerLine.evalAtPoint(ss(i));

                //normal
                Vector<2> centerLineNormal = centerLine.normalAtPoint(ss(i));
                Vector<2> trajPoint = centerLinePoint + dd(i)*centerLineNormal; //the trajectoray point is the centerLinePoint plus d times the normal (should be oriented the right way and normalized)

                toAdd.position.x = trajPoint(0);
                toAdd.position.y = trajPoint(1);

                // tangent
                Vector<2> centerLineTangent = centerLine.tangentAtPoint(ss(i));

                toAdd.directory.x = centerLineTangent(0);
                toAdd.directory.y = centerLineTangent(1);

                // velocity (norm tangent times derivative d/dt s(t))
                toAdd.velocity = centerLine.normTangentAtPoint(ss(i))*ss_d(i);

                // side
                toAdd.distanceToMiddleLane = dd(i);

            } else if (ss(i) > s_end)
            {
                //the point is behind the trajectory --> linear extrapolation
                //point on the center line
                Vector<2> centerLinePoint = centerLine.evalAtPoint(s_end);

                //normal
                Vector<2> centerLineNormal = centerLine.normalAtPoint(s_end);

                //tangent
                Vector<2> centerLineTangent = centerLine.tangentAtPoint(s_end);


                Vector<2> trajPoint = centerLinePoint + (ss(i)-s_end)*centerLineTangent + dd(i)*centerLineNormal;

                toAdd.directory.x = trajPoint(0);
                toAdd.directory.y = trajPoint(1);

                toAdd.directory.x = centerLineTangent(0);
                toAdd.directory.y = centerLineTangent(1);

                toAdd.velocity = centerLine.normTangentAtPoint(ss(i))*ss_d(i);

                // side
                toAdd.distanceToMiddleLane = dd(i);

            } else
            {
                // case not considered: use zeros
                // this should not be: throw error
                toAdd.directory.x = 0;
                toAdd.directory.y = 0;

                toAdd.directory.x = 0;
                toAdd.directory.y = 0;

                toAdd.velocity = 0;

                // side
                toAdd.distanceToMiddleLane = 0;
            }

            // add to Trajectory
            trajectoryOut.push_back(toAdd);
        }


        return trajectoryOut;
    }

    /**
     * Projects the trajectory on the unique Bezier Curve that is defined by the k points in points (points2d<k> struct points). Also return tangent + velocity
     * l = diastance between two successive points
     */

    street_environment::Trajectory projectOntoLineSegments(lms::math::polyLine2f road, float lSample)
    {

        // initialize
        // generate output
        street_environment::Trajectory trajectoryOut;
        street_environment::TrajectoryPoint toAdd;

        Poly<3> poly_s_d = mPtr_s->differentiate(); //first derivative
        Poly<4> poly_d_d = mPtr_d->differentiate();

        int nPoints = (road.points().size());
        float k = (1.0 * nPoints);
        float l = road.points()[0].distance(road.points()[1]);

        float s_end = l*(k-1); //as there are (k-1) segemnts of length l between k points

        float max_velocity = S.v1;
        float min_velocity = S.v0;

        if (S.v1 < S.v0)
        {
            max_velocity = S.v0;
            min_velocity = S.v1;
        }


        float dt = lSample/max_velocity;
        const int m = ceil(s_end/(dt*min_velocity));
        //std::cout << "m for first eval: " << m << " ,  l: " << l << " ,lSample: " << lSample << ",  s_end " << s_end <<  std::endl;
        //std::cout << "dt for first eval: " << dt << std::endl;


        float t_local = 0;
        float s_local = mPtr_s->evalAtPoint(t_local);
        float d_local = mPtr_d->evalAtPoint(t_local);


        float s_d_local = poly_s_d.evalAtPoint(t_local);
        float d_d_local = poly_d_d.evalAtPoint(t_local);

        for (int i = 0; i <m ; i++)
        {

            t_local = dt*i;
            if (t_local <= tend) {
                s_local = mPtr_s->evalAtPoint(t_local);
                d_local = mPtr_d->evalAtPoint(t_local);
                s_d_local = poly_s_d.evalAtPoint(t_local);
            }else
            {
                s_local = mPtr_s->evalAtPoint(tend) + S.v1*(t_local-tend);
                s_d_local = S.v1;
                d_local = D.d1;
                d_d_local = 0;
            }

            //std::cout << "s_local: " << s_local << std::endl;

            if ((s_local > 0) && (s_local < s_end) && (t_local > 0))
            {

                lms::math::vertex2f centerLinePoint = road.interpolateAtDistance(s_local);


                //normal
                lms::math::vertex2f centerLineNormal = road.interpolateNormalAtDistance(s_local);
                lms::math::vertex2f trajPoint = centerLinePoint + (centerLineNormal *
                                                                   d_local); //the trajectoray point is the centerLinePoint plus d times the normal (should be oriented the right way and normalized)

                toAdd.position = trajPoint;

                // tangent
                lms::math::vertex2f centerLineTangent = road.interpolateTangentAtDistance(s_local);

                lms::math::vertex2f directionTraj =
                        (centerLineTangent * s_d_local) + (centerLineNormal * d_d_local);
                toAdd.directory = directionTraj.normalize();

                // velocity (norm tangent times derivative d/dt s(t))
                toAdd.velocity = sqrtf(pow(s_d_local, 2) + pow(d_d_local, 2));

                // side
                toAdd.distanceToMiddleLane = d_local;


                if (centerLineNormal.length() != 1) {
                    //std::cout << "normal length wrong " << centerLineNormal.length() << std::endl;
                }
                if (centerLineNormal.length() != 1) {
                    //std::cout << "tangent length wrong " << centerLineTangent.length() << std::endl;
                }

                if (d_local > 0.3) {
                    //std::cout << "d_local too big: " << d_local << ",   d0: " << D.d0 << ",  d1: " << D.d1 << ",  d0d: " << D.d0d << ",  d0dd: " << D.d0dd << std::endl;
                    //std::cout << " centerLinePoint" << centerLinePoint << std::endl;
                    //std::cout << " s loc " << s_local << ",  d loc " << d_local << ",  t_end " << tend << ",  t_local " << t_local << std::endl ;
                    //std::cout << "tangent " << centerLineTangent << std::endl;
                    //std::cout << "normal " << centerLineNormal << std::endl;
                    //std::cout << std::endl << std::endl;
                }

                if (d_local < -0.3) {
                    //std::cout << "d_local too small: " << d_local << std::endl;
                }

                if (std::isnan(toAdd.position.x)) {
                    //std::cout << "lhadfs x" << std::endl;
                    //std::cout << " centerLinePoint" << centerLinePoint << std::endl;
                    //std::cout << " s loc" << s_local << "d loc " << d_local << std::endl;
                    //std::cout << "tangent " << centerLineTangent << std::endl;
                    //std::cout << "normal " << centerLineNormal << std::endl;
                }
                if (std::isnan(toAdd.position.y)) {
                    //std::cout << "lhadfs y" << std::endl;
                }
                if (std::isnan(toAdd.directory.x)) {
                    //std::cout << "lhadfs dx" << std::endl;
                }
                if (std::isnan(toAdd.directory.y)) {
                    //std::cout << "lhadfs dy" << std::endl;
                }
            }


            // add to Trajectory
            trajectoryOut.push_back(toAdd);
        }


        return trajectoryOut;
    }


    street_environment::Trajectory projectOntoLineSegments(lms::math::polyLine2f road, float lSample, float t_offset)
    {

        float s_offset = 0;

        if (t_offset > tend)
        {
            s_offset = mPtr_s->evalAtPoint(tend) + (t_offset-tend)*S.v1;
        }else
        {
            s_offset = mPtr_s->evalAtPoint(t_offset);
        }



        // initialize
        // generate output
        street_environment::Trajectory trajectoryOut;
        street_environment::TrajectoryPoint toAdd;

        Poly<3> poly_s_d = mPtr_s->differentiate(); //first derivative
        Poly<4> poly_d_d = mPtr_d->differentiate();

        int nPoints = (road.points().size());
        float k = (1.0 * nPoints);
        float l = road.points()[0].distance(road.points()[1]);

        float s_end = l*(k-1); //as there are (k-1) segemnts of length l between k points

        float max_velocity = S.v1;
        float min_velocity = S.v0;

        if (S.v1 < S.v0)
        {
            max_velocity = S.v0;
            min_velocity = S.v1;
        }


        float dt = lSample/max_velocity;
        const int m = ceil(s_end/(dt*min_velocity));
        //std::cout << "m for alternate eval: " << m << std::endl;
        //std::cout << "dt for alternate eval: " << dt << std::endl;


        float t_local = 0 + t_offset;
        //std::cout << "new eval with offset in t: " << t_offset << ",  offset in s: " << s_offset << std::endl;

        float s_local = mPtr_s->evalAtPoint(t_local)-s_offset;
        float d_local = mPtr_d->evalAtPoint(t_local);


        float s_d_local = poly_s_d.evalAtPoint(t_local);
        float d_d_local = poly_d_d.evalAtPoint(t_local);

        for (int i = 0; i <m ; i++)
        {

            t_local = dt*i + t_offset;
            if (t_local <= tend) {
                s_local = mPtr_s->evalAtPoint(t_local) - s_offset;
                d_local = mPtr_d->evalAtPoint(t_local);
                s_d_local = poly_s_d.evalAtPoint(t_local);
            }else
            {
                s_local = mPtr_s->evalAtPoint(tend) + S.v1*(t_local-tend) - s_offset;
                s_d_local = S.v1;
                d_local = D.d1;
                d_d_local = 0;
            }

            //std::cout << "s_local: " << s_local << ",  t_local: " << t_local << std::endl;

            if ((s_local > 0) && (s_local < s_end) && (t_local > 0))
            {

                lms::math::vertex2f centerLinePoint = road.interpolateAtDistance(s_local);


                //normal
                lms::math::vertex2f centerLineNormal = road.interpolateNormalAtDistance(s_local);
                lms::math::vertex2f trajPoint = centerLinePoint + (centerLineNormal *
                                                                   d_local); //the trajectoray point is the centerLinePoint plus d times the normal (should be oriented the right way and normalized)

                toAdd.position = trajPoint;

                // tangent
                lms::math::vertex2f centerLineTangent = road.interpolateTangentAtDistance(s_local);

                lms::math::vertex2f directionTraj =
                        (centerLineTangent * s_d_local) + (centerLineNormal * d_d_local);
                toAdd.directory = directionTraj.normalize();

                // velocity (norm tangent times derivative d/dt s(t))
                toAdd.velocity = sqrtf(pow(s_d_local, 2) + pow(d_d_local, 2));

                // side
                toAdd.distanceToMiddleLane = d_local;


                if (centerLineNormal.length() != 1) {
                    //std::cout << "normal length wrong " << centerLineNormal.length() << std::endl;
                }
                if (centerLineNormal.length() != 1) {
                    //std::cout << "tangent length wrong " << centerLineTangent.length() << std::endl;
                }

                if (d_local > 0.3) {
                    //std::cout << "d_local too big: " << d_local << ",   d0: " << D.d0 << ",  d1: " << D.d1 << ",  d0d: " << D.d0d << ",  d0dd: " << D.d0dd << std::endl;
                    //std::cout << " centerLinePoint" << centerLinePoint << std::endl;
                    //std::cout << " s loc " << s_local << ",  d loc " << d_local << ",  t_end " << tend << ",  t_local " << t_local << std::endl ;
                    //std::cout << "tangent " << centerLineTangent << std::endl;
                    //std::cout << "normal " << centerLineNormal << std::endl;
                    //std::cout << std::endl << std::endl;
                }

                if (d_local < -0.3) {
                    //std::cout << "d_local too small: " << d_local << std::endl;
                }

                if (std::isnan(toAdd.position.x)) {
                    //std::cout << "lhadfs x" << std::endl;
                    //std::cout << " centerLinePoint" << centerLinePoint << std::endl;
                    //std::cout << " s loc" << s_local << "d loc " << d_local << std::endl;
                    //std::cout << "tangent " << centerLineTangent << std::endl;
                    //std::cout << "normal " << centerLineNormal << std::endl;
                }
                if (std::isnan(toAdd.position.y)) {
                    //std::cout << "lhadfs y" << std::endl;
                }
                if (std::isnan(toAdd.directory.x)) {
                    //std::cout << "lhadfs dx" << std::endl;
                }
                if (std::isnan(toAdd.directory.y)) {
                    //std::cout << "lhadfs dy" << std::endl;
                }
            }


            // add to Trajectory
            trajectoryOut.push_back(toAdd);
        }


        return trajectoryOut;
    }
/**
 * @brief work in progress: Do not USE!!!!
 */
    /* template<size_t m>
     RoadData Trajectory::convertPointsToRoadData(const points2d<m> pointsIn){

         RoadData roadDataOut;

         if (pointsIn.x(0) > 0){
             // the points are all infront of the car so i extrapolate linearly
             T dx = pointsIn.x(1) - pointsIn.x(0);
             T dy = pointsIn.y(1) - pointsIn.y(0);

             if (dx == 0){
                 dx = 0.001;
             }

             roadDataOut.y0 = pointsIn.y(0) + dy/dx*(-pointsIn.x(0));
             roadDataOut.phi = atan(dy/dx);
         }
     }*/


    /**
     * empty constructor
     */
    Trajectory()
    {

    }

    Trajectory(const float& _v1, const float& _d1, const float & _safetyS, const float& _safetyD, const RoadData& _roadData1, const std::vector<ObstacleData>& _obstacles, float _tend, const CoeffCtot& _coeffCtot);


    /**
     * Output to iostream
     */
    friend std::ostream& operator<<(std::ostream& os,  Trajectory& trajectory)
    {
        os << "Trajectory with the following propteries:" << std::endl
        << "time for Trajectory: " << trajectory.tend << std::endl
        << "Polynomials in Frenet space:" <<std::endl
        << "s: \t" << *trajectory.mPtr_s << std::endl
        << "d: \t" << *trajectory.mPtr_d << std::endl
        << "total value of the cost function: " << std::setprecision(30) << trajectory.ctot() <<std::endl
        << "this trajectory is colliding:" << trajectory.doesCollide() << std::endl
        << "this traj. is physically drivable: " << trajectory.drivable << std::endl;
        return os;

    }

private:
    /*std::unique_ptr<Poly<5>> mPtr_d;
    std::unique_ptr<Poly<4>> mPtr_s;*/


    Poly<5>* mPtr_d; //pointer to the poly. object for the d (lateral) direction in Frenet space
    Poly<4>* mPtr_s; //pointer to the poly. object for the s (longitudinal) direction in Frenet space

    CoeffCtot coeffCtot1; //coefficients of the cost function (see also types.h)

    float kappa; //curvature of the road (approx. as a circle)

    float ctot_value; //value of the total cost function
    bool ctot_alreadyCalc; //was the value already calculated

    RoadData roadData1; //data of the road (see also types.h)

    std::vector<ObstacleData> obstacles; //vector: each obstacleData is the data corresp. to one obstacle (see also types.h)

    bool collision = true; //is there a collision
    bool collisionDetected = false; //was collision already checked

    bool drivable = true; //is the traj. physically (no obstacles considered)
    bool drivabilityDetected = true; //was drivability already calculated

    int nSamplesCollisionAndDrivabilityDetection = 300;


    float safetyS; //safety distance in s (longitudinal) direction: IMPORTANT: Both cars are assumed as points so this must at least include half their lengths
    float safetyD; //safety distance in d (lateral) direction: IMPORTANT: this is measured from the center line

    float Jtd(); //function that returns the smoothness-functional for the d direction

    float Jts(); //function that returns the smoothness-functional for the s direction

    S_initialCond S;
    D_initialCond D;

};


#endif //PROJECT_TRAJECTORY_H
