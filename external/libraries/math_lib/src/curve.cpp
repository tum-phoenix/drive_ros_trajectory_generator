#include "lms/math/curve.h"
#include <cstdlib>

namespace lms {
namespace math {

void bresenhamLine(int x0, int y0, int x1, int y1,std::function<bool(int,int)> found){
  int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = dx+dy, e2; /* error value e_xy */

  for(;;){  /* loop */
    if(!found(x0,y0))
        break;
    if (x0==x1 && y0==y1) break;
    e2 = 2*err;
    if (e2 > dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
    if (e2 < dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
  }
}

void bresenhamLine(int x0, int y0, int x1, int y1, std::vector<int> &vX, std::vector<int> &vY){
    bresenhamLine(x0,y0,x1,y1,[&vX,&vY](int x,int y){
        vX.push_back(x);
        vY.push_back(y);
        return true;
    });
}
}  // namespace math
}  // namespace lms
