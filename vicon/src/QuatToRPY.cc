#include <cmath>
#include "QuatToRPY.h"

void QuatToRPY(double qx, double qy, double qz, double qw,
               double &roll, double &pitch, double &yaw)
{
  double sqw = qw * qw;
  double sqx = qx * qx;
  double sqy = qy * qy;
  double sqz = qz * qz;
  double norm = sqrt(sqw + sqx + sqy + sqz);
  
  // Normalize to get unit quaternion
  qw /= norm;
  qx /= norm;
  qy /= norm;
  qz /= norm;
  
  // Now using Tait-Bryan angles Rz(yaw)*Ry(pitch)*Rx(roll) <=>
  // R_{body->world} <=> q
  roll = atan2(2*qw*qx + 2*qy*qz, sqw - sqx
               - sqy + sqz);
  pitch = asin(2*qw*qy - 2*qx*qz);
  yaw = atan2(2*qw*qz + 2*qx*qy, sqw + sqx
              - sqy - sqz);        
}
