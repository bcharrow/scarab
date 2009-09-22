//
// Quaternion Class
//
#ifndef CQuat_h
#define CQuat_h

#include <math.h>

#include <iostream>
const double TO_HALF_RAD = 3.14159265f / 360.0f;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class CQuat
{
public:
  double x,y,z,w;

  CQuat( ) : x(0), y(0), z(0), w(1)
  {
  }

  CQuat( double fx, double fy, double fz, double fw ) : x(fx), y(fy), z(fz), w(fw)
  {
  }

  // No rotation
  void Reset( )
  {
    x = 0;
    y = 0;
    z = 0;
    w = 1;
  }

  // Set Quat from axis-angle
  void SetAxis( double radians, double fX, double fY, double fZ )
  {
    //double HalfAngle = degrees * TO_HALF_RAD; // Get half angle in radians from angle in degrees
    double HalfAngle = radians/2.0;
    double sinA = (double)sin( HalfAngle ) ;
    w = (double)cos( HalfAngle );
    x = fX * sinA;
    y = fY * sinA;
    z = fZ * sinA;
  }

  CQuat Invert( ) const
  {
    return CQuat( -x, -y, -z, w );
  }

  void RotateVector(double &vx, double &vy, double &vz)
  {
    double v1 = vx;
    double v2 = vy;
    double v3 = vz;
    double t2, t3, t4, t5, t6, t7, t8, t9, t10;

    t2 =   w*x;
    t3 =   w*y;
    t4 =   w*z;
    t5 =  -x*x;
    t6 =   x*y;
    t7 =   x*z;
    t8 =  -y*y;
    t9 =   y*z;
    t10 = -z*z;
    vx = 2*( (t8 + t10)*v1 + (t6 -  t4)*v2 + (t3 + t7)*v3 ) + v1;
    vy = 2*( (t4 +  t6)*v1 + (t5 + t10)*v2 + (t9 - t2)*v3 ) + v2;
    vz = 2*( (t7 -  t3)*v1 + (t2 +  t9)*v2 + (t5 + t8)*v3 ) + v3;
  }

  // Note that order matters with concatenating Quaternion rotations
  inline CQuat operator* (const CQuat &b) const
  {
    CQuat r;

    r.w = w*b.w - x*b.x  -  y*b.y  -  z*b.z;
    r.x = w*b.x + x*b.w  +  y*b.z  -  z*b.y;
    r.y = w*b.y + y*b.w  +  z*b.x  -  x*b.z;
    r.z = w*b.z + z*b.w  +  x*b.y  -  y*b.x;

    return r;
  }
  
  // You could add an epsilon to this equality test if needed
  inline bool operator== ( const CQuat &b ) const
  {
    return (x == b.x && y == b.y && z == b.z && w == b.w);
  }

  int IsIdentity( ) const
  {
    return (x == 0.0f && y == 0.0f && z == 0.0f && w==1.0f);
  }

  // Can be used the determine Quaternion neighbourhood
  double Dot( const CQuat& a ) const
  {
    return x * a.x + y * a.y + z * a.z + w * a.w;
  }

  // Scalar multiplication
  CQuat operator*( double s ) const
  {
    return CQuat(x * s, y * s, z * s, w * s );
  }

  CQuat inverse( ) const
  {
    double norm = this->Dot(*this);
    return CQuat(-x/norm, -y/norm, -z/norm, w/norm);
  }

  // Addition
  CQuat operator+ ( const CQuat& b ) const
  {
    return CQuat( x + b.x, y + b.y, z + b.z, w + b.w );
  }

  // ------------------------------------
  // Simple Euler Angle to Quaternion conversion, this could be made faster
  // ------------------------------------
  /*
    void FromEuler( double rx, double ry, double rz )
    {
    CQuat qx(-rx, CVec3( 1, 0, 0 ) );
    CQuat qy(-ry, CVec3( 0, 1, 0 ) );
    CQuat qz(-rz, CVec3( 0, 0, 1 ) );
    qz = qy * qz;
    *this = qx * qz;
    }
  */

  // ------------------------------------
  // Quaternions store scale as well as rotation, but usually we just want rotation, so we can normalize.
  // ------------------------------------
  int Normalize( )
  {
    double lengthSq = x * x + y * y + z * z + w * w;

    if (lengthSq == 0.0 ) return -1;
    if (lengthSq != 1.0 )
      {
        double scale = ( 1.0 / sqrtf( lengthSq ) );
        x *= scale;
        y *= scale;
        z *= scale;
        w *= scale;
        return 1;
      }
    return 0;
  }

  // ------------------------------------
  // Creates a value for this Quaternion from spherical linear interpolation
  // t is the interpolation value from 0 to 1
  // ------------------------------------
  void Slerp(const CQuat& a, const CQuat& b, double t)
  {
    double w1, w2;

    double cosTheta = a.Dot(b);
    double theta    = (double)acos(cosTheta);
    double sinTheta = (double)sin(theta);

    if( sinTheta > 0.001f )
      {
        w1 = double( sin( (1.0f-t)*theta ) / sinTheta);
        w2 = double( sin( t*theta) / sinTheta);
      } else {
        // CQuat a ~= CQuat b
        w1 = 1.0f - t;
        w2 = t;
      }

    *this = a*w1 + b*w2;
  }

  // ------------------------------------
  // linearly interpolate each component, then normalize the Quaternion
  // Unlike spherical interpolation, this does not rotate at a constant velocity,
  // although that's not necessarily a bad thing
  // ------------------------------------
  void NLerp( const CQuat& a, const CQuat& b, double w2)
  {
    double w1 = 1.0f - w2;

    *this = a*w1 + b*w2;
    Normalize();
  }

  // ------------------------------------
  // Set a 4x4 matrix with the rotation of this Quaternion
  // ------------------------------------
  void inline ToMatrix( double mf[16] ) const
  {
    double x2 = 2.0f * x,  y2 = 2.0f * y,  z2 = 2.0f * z;

    double xy = x2 * y,  xz = x2 * z;
    double yy = y2 * y,  yw = y2 * w;
    double zw = z2 * w,  zz = z2 * z;

    mf[ 0] = 1.0f - ( yy + zz );
    mf[ 1] = ( xy - zw );
    mf[ 2] = ( xz + yw );
    mf[ 3] = 0.0f;

    double xx = x2 * x,  xw = x2 * w,  yz = y2 * z;

    mf[ 4] = ( xy +  zw );
    mf[ 5] = 1.0f - ( xx + zz );
    mf[ 6] = ( yz - xw );
    mf[ 7] = 0.0f;

    mf[ 8] = ( xz - yw );
    mf[ 9] = ( yz + xw );
    mf[10] = 1.0f - ( xx + yy );  
    mf[11] = 0.0f;  

    mf[12] = 0.0f;  
    mf[13] = 0.0f;   
    mf[14] = 0.0f;   
    mf[15] = 1.0f;
  }

  void ToEuler(double &roll, double &pitch, double &yaw) 
  {
    double rx, ry, rz, theta;
    theta = 2*acos(w);
    rx = x/sin(theta/2);
    ry = y/sin(theta/2);
    rz = z/sin(theta/2);

    double c = cos(theta);
    double s = sin(theta);
    double t = 1-c;

    /*
      if ((rx*ry*t + rz*s) > 0.998) { // north pole singularity detected
      roll = 2*atan2(rx*sin(theta/2),cos(theta/2));
      pitch = M_PI/2;
      yaw = 0;
      std::cout << "LOCK" << std::endl;
      }
      else if ((rx*ry*t + rz*s) < -0.998) { // south pole singularity detected
      roll = -2*atan2(rx*sin(theta/2),cos(theta/2));
      pitch = -M_PI/2;
      yaw = 0;

      std::cout << "LOCK2" << std::endl;
      } 
      else {
      roll = atan2(ry * s- rx * rz * t , 1 - (ry*ry+ rz*rz ) * t);
      pitch = asin(rx * ry * t + rz * s) ;
      yaw = atan2(rx * s - ry * rz * t , 1 - (rx*rx + rz*rz) * t);
      }
    */

    /*
      if ((-rx*rz*t + ry*s) > 0.998) { // north pole singularity detected
      roll = 2*atan2(rx*sin(theta/2),cos(theta/2));
      pitch = M_PI/2;
      yaw = 0;
      std::cout << "LOCK" << std::endl;
      }
      else if ((-rx*rz*t + ry*s) < -0.998) { // south pole singularity detected
      roll = -2*atan2(rx*sin(theta/2),cos(theta/2));
      pitch = -M_PI/2;
      yaw = 0;

      std::cout << "LOCK2" << std::endl;
      } 
      else 
    */
    {
      roll = atan2(ry*rz*t + rx*s, 1 - (rx*rx + ry*ry)*t);
      pitch = asin(-(rx*rz*t - ry*s));
      yaw = atan2(rx*ry*t + rz*s, 1 - (ry*ry + rz*rz)*t);
    }

    //std::cout << "RPY " << roll << " " << pitch << " " << yaw << std::endl;
  }

  /*
    void ToEuler(double &roll, double &pitch, double &yaw)
    {
    double squ;
    double sqx;
    double sqy;
    double sqz;

    this->Normalize();

    squ = this->w * this->w;
    sqx = this->x * this->x;
    sqy = this->y * this->y;
    sqz = this->z * this->z;

    this->Normalize();

    // Roll
    roll = atan2(2 * (this->y*this->z + this->w*this->x), squ - sqx - sqy + sqz);

    // Pitch
    pitch = asin(-2 * (this->x*this->z - this->w * this->y));

    // Yaw
    yaw = atan2(2 * (this->x*this->y + this->w*this->z), squ + sqx - sqy - sqz);
    }
  */

#ifdef OLD
  void ToEuler(double &roll, double &pitch, double &yaw)
  {
    double mf[16];
    this->ToMatrix(mf);

    /*
     *   [m00 m01 m02]
     *   [m10 m11 m12]
     *   [m20 m21 m22]
     *
     *   [m0 m1 m2] m3
     *   [m4 m5 m6] m7
     *   [m8 m9 m10] m11
     *   m12 m13 m14 m15
     */

    // Assuming the angles are in radians.
    if (mf[4] > 0.998) { // singularity at north pole
      yaw = atan2(mf[2],mf[10]);
      pitch = M_PI/2;
      roll = 0;
    }
    else if (mf[4] < -0.998) { // singularity at south pole
      yaw = atan2(mf[2],mf[10]);
      pitch = -M_PI/2;
      roll = 0;
    }
    else {
      yaw = atan2(-mf[8],mf[0]);
      pitch = atan2(-mf[6],mf[5]);
      roll = asin(mf[4]);
    }


  }
#endif

};

#endif
