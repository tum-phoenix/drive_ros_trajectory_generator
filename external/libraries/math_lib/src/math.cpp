#include "lms/math/math.h"
#include <cmath>
namespace lms {
namespace math {

float limitAngle_0_2PI(float angle){
    while(angle > 2*M_PI){
        angle -= 2*M_PI;
    }
    while(angle < 0){
        angle += 2*M_PI;
    }
    return angle;
}
float limitAngle_nPI_PI(float angle){
    while(angle > M_PI){
        angle -= 2*M_PI;
    }
    while(angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}
}
}
