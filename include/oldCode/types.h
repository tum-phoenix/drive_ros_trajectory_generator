//
// Created by Lukas Koestler on 23.10.15.
//

#ifndef PROJECT_TYPES_H
#define PROJECT_TYPES_H

#include <Eigen/Dense>

template<int rows, int cols>
using Matrix = Eigen::Matrix<float, rows, cols>;

template<int rows>
using Vector = Matrix<rows, 1>;
/**
 * @brief The RoadData struct captures the current state of the road and the car on it
 */
struct RoadData{
    float y0; // intercept of the center line and the y-axis of the car coordinates (different sign than street coordinates)
    float phi; //angle of the centerline relative to the car x-axis
    float w; // angular velocity of the car (z axis positive rotation)
    float vx0; //initial velocity of the car in x (car) direction
    float ax0; //initial acceleration of the car in x (car) direction
    float kappa; //curvature of the road (approximated as a circle)

};
/**
 * @brief The obstacleData struct caprutres the data of one obstacle on the road
 */
struct ObstacleData{
    float s0; // initial distance along the road of the obstacle to the car
    float v0; // initial velocity of the obstacle (NOT REALTIVE TO THE CAR) along the road
    bool leftLane; //lane of the obstacle
};
/**
 * @brief The CoeffCtot struct coefficients for the cost function
 */
struct CoeffCtot
{
    float kj; //cost of the non-smoothness (kj high --> gives smooth traj.)
    float kT; //cost of the time the traj. needs (kT high --> gives fast (non-smooth) tral.)
    float kd; //cost. of the lateral direction
    float ks; //cost. of the longitudinal direction
};

typedef struct
{
    // captures the initial conditions for the polynomial in s (longitudinal) Frenet space
    float v0; //initial velocity in s-direction (!= vx0)
    float a0; //initial acceleration in s-direction (!= ax0)
    float v1; //velocity in s direction at the end of the traj.
} S_initialCond;

typedef struct
{
    // captures the initial conditions for the polynomial in d (lateral) Frenet space
    float d0; //initial offset in d-direction (!= y0, and sign is different from y0)
    float d0d; //initial velocity in d-direction
    float d0dd; //initial acceleration in d-direction
    float d1; //offset in d-direction at the end of the trj.
} D_initialCond;

template<size_t N>
struct  points2d
{
    Vector<N> x;
    Vector<N> y;
};

#endif //PROJECT_TYPES_H
