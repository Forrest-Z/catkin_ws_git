#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
using namespace std;

void vehframe2cartesian(float veh_x, float veh_y, float theta, float& cartesian_x, float& cartesian_y){
  float alpha = atan2(veh_y, veh_x);
  float length = sqrt(pow(veh_x,2)+ pow(veh_y,2));

  cartesian_x = -1*length*sin(alpha+theta);
  cartesian_y = length*cos(alpha+theta);
}

#endif  // HELPERS_H