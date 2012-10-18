#include <ros/ros.h>
#include <string>
#include <stdlib.h>
#include <math.h>
#include "AcronameMotor.h"

using namespace std;

AcronameMotor::AcronameMotor()
{
  this->error = aErrNone;  
  
  /* Get the references to the aIO and aStem library objects. */
  if (error == aErrNone)
    aIO_GetLibRef(&ioLib, &error);
  if (error == aErrNone)
    aStem_GetLibRef(&stemLib, &error);

  if( error != aErrNone )
    printf("Problem getting librefs\n");

}

void AcronameMotor::
SetupPort(const string &portname, const long &portspeed)
{
  /* Build a link stream to communicate serially with the stem. */
  if (error == aErrNone)
    aStream_CreateSerial(ioLib, 
                         portname.c_str(), 
                         portspeed,                  
                         &linkStream, 
                         &error);

  if( error != aErrNone ) {
    printf("Problem forming link to Moto... ");
    
    switch(error) {
    case aErrNotFound:
      printf("port not found!\n");
      break;
    case aErrRange:
      printf("baud rate not available\n");
      break;
    case aErrOverrun:
      printf("buffer overrun\n");
    default:
      printf("unknown problem\n");
    };
  }

  if (error == aErrNone)
    aStem_SetStream(stemLib, linkStream, kStemModuleStream, &error);

}

void AcronameMotor::
SetupChannels(int left, int left_dir, int right, int right_dir)
{
  left_channel = left;
  right_channel = right;

  unsigned char mode_flags = 0;

  if(left_dir > 0) {
    mode_flags = 0;
    mode_flags |= (1 << aMOTION_PWMFLAG_INVPID);
  }
  else {
    mode_flags = 0;
    mode_flags |= (1 << aMOTION_PWMFLAG_INVPWM);
  }

  error = aMotion_SetMode(stemLib,
                          aMODULE,
                          left_channel,
                          aMOTION_MODE_ENCVEL,
                          mode_flags);

  if(right_dir > 0) {
    mode_flags = 0;
    mode_flags |= (1 << aMOTION_PWMFLAG_INVPID);
  }
  else {
    mode_flags = 0;
    mode_flags |= (1 << aMOTION_PWMFLAG_INVPWM);
  }

  error = aMotion_SetMode(stemLib,
                          aMODULE,
                          right_channel,
                          aMOTION_MODE_ENCVEL,
                          mode_flags);

  
}

int AcronameMotor::
SetPWMFreq(unsigned char paramH, unsigned char paramL)
{
  // See acroname's site for details on parameters
  // http://www.acroname.com/brainstem/ref/ref.html#Commands/cmdMO_CFG.html
  int val = 0;

  // PWM period = [(4^paramH)*(paramL+1)*0.1us]
  // PWM freq = 1 / PWM period
  // PWM resolution = log(40MHz / PWM freq)/log(2)

  double period = pow(4, paramH) * (paramL + 1) * 1e-7;
  double freq = 1 / period;
  double resolution = log(40 * 10e6 / freq) / log(2);

  paramH = 0;
  paramL = 255;
  val = paramL;
  val |= (paramH << 8);

  ROS_INFO("paramL: %d, paramH: %d, PWM Freq = %.2f Resolution = %0.1f",
           paramL, paramH, freq, resolution);
  
  error = aMotion_SetParam(stemLib, aMODULE, left_channel,
                           aMOTION_PARAM_PWMFREQ, val);
  

  if(error != aErrNone) {
    return -1;
  }

  return 0;
}

int16_t fixed_width(double p) {
  // Convert double precision number to 16-bit fixed precision number.  High
  // bit is sign bit, low order 5 bits are fractional, middle bits are
  // integral.
  int16_t integral = static_cast<int16_t>(p);
  int16_t high_mask = 0x7fe0;
  int16_t low_mask = 0x001f;
  int16_t fractional = round((p - integral) * 32.0);
  int16_t result = (high_mask & (integral << 5)) | (low_mask & fractional);
  return result;
}


void AcronameMotor::
SetupPID(double p, double i, double d, double period)
{

  // Set motor PID params.  The resolution step size is fixed width, see
  // documentaiton for details
  error = aMotion_SetParam(stemLib, aMODULE, left_channel, aMOTION_PARAM_P,
                           fixed_width(p));
  if (error != aErrNone) { ROS_WARN("Error setting param!"); }

  error = aMotion_SetParam(stemLib, aMODULE, left_channel, aMOTION_PARAM_I,
                           fixed_width(i));
  if (error != aErrNone) { ROS_WARN("Error setting param!"); }

  error = aMotion_SetParam(stemLib, aMODULE, left_channel, aMOTION_PARAM_D,
                           fixed_width(d));
  if (error != aErrNone) { ROS_WARN("Error setting param!"); }

  error = aMotion_SetParam(stemLib, aMODULE, right_channel, aMOTION_PARAM_P,
                           fixed_width(p));
  if (error != aErrNone) { ROS_WARN("Error setting param!"); }

  error = aMotion_SetParam(stemLib, aMODULE, right_channel, aMOTION_PARAM_I,
                           fixed_width(i));
  if (error != aErrNone) { ROS_WARN("Error setting param!"); }

  error = aMotion_SetParam(stemLib, aMODULE, right_channel, aMOTION_PARAM_D,
                           fixed_width(d));
  if (error != aErrNone) { ROS_WARN("Error setting param!"); }

  /* Set the Period value. The increment value that can be stored is 
   * in 0.1msec increments. Depending on the Moto firmware, the lowest 
   * possible value is 1msec, which is a value of 10. 
   */
  error = aMotion_SetParam(stemLib, aMODULE, left_channel,
                           aMOTION_PARAM_PERIOD,
                           (int)(round(period/0.0001)));
  if (error != aErrNone) { ROS_WARN("Error setting param!"); }

  error = aMotion_SetParam(stemLib, aMODULE, right_channel,
                           aMOTION_PARAM_PERIOD,
                           (int)(round(period/0.0001)));
  if (error != aErrNone) { ROS_WARN("Error setting param!"); }
}

int AcronameMotor::
SetVel(short int left_vel, short int right_vel)
{
  error = aMotion_SetValue(stemLib, aMODULE, left_channel, left_vel);
  if (error != aErrNone) { ROS_WARN("Error setting left velocity!"); }

  error = aMotion_SetValue(stemLib, aMODULE, right_channel, right_vel);
  if (error != aErrNone) { ROS_WARN("Error setting right velocity!"); }

  if(error == aErrNone)
    return 0;

  return -1;
}

int AcronameMotor::
GetVel(short int &left_vel, short int &right_vel)
{
  error = aMotion_GetPIDInput(stemLib, aMODULE, left_channel, &left_vel);

  error = aMotion_GetPIDInput(stemLib, aMODULE, right_channel, &right_vel);

  if(error == aErrNone)
    return 0;

  return -1;
}

AcronameMotor::
~AcronameMotor()
{
  aStem_ReleaseLibRef(stemLib, NULL);
  aIO_ReleaseLibRef(ioLib, NULL);

}

bool AcronameMotor::
ok()
{
  return (error == aErrNone);
}
