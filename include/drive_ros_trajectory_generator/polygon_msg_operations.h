#ifndef POLYGON_MSG_OPERATIONS_H
#define POLYGON_MSG_OPERATIONS_H

#include <drive_ros_msgs/DrivingLine.h>

inline float compute_polynomial_at_location (const drive_ros_msgs::DrivingLineConstPtr& msg, float X) {
  float Y = 0.f;
  for(int i = 0; i <= msg->polynom_order; i++)
    Y += msg->polynom_params.at(i)*std::pow(X, i);
  return Y;
}

inline float derive_polynomial_at_location (const drive_ros_msgs::DrivingLineConstPtr& msg, float X) {
  float deriv = 0.f;
  for(int i = 0; i <= msg->polynom_order-1; i++)
    deriv += msg->polynom_params.at(i+1)*(i+1)*std::pow(X, i);
  return deriv;
}

inline float compute_polynomial_integral(const drive_ros_msgs::DrivingLineConstPtr& msg, float x_start, float x_end,
                                         float x_step=0.1f) {
  float length = 0.f;
  float last_y = compute_polynomial_at_location(msg, x_start);
  float curr_y = 0.f;
  for (float x = x_start; x < x_end; x+=x_step) {
    curr_y = compute_polynomial_at_location(msg, x);
    length += std::sqrt(std::pow(x_step, 2)+std::pow(curr_y-last_y, 2));
    last_y = curr_y;
  }
  return length;
}

#endif // POLYGON_MSG_OPERATIONS_H
