#include <ros/ros.h>
#include <string>
#include <stdlib.h>
#include <math.h>
#include "AcronameMotor.h"

using namespace std;

AcronameMotor::AcronameMotor() {
  this->error = aErrNone;  
  
  /* Get the references to the aIO and aStem library objects. */
  if (error == aErrNone)
    aIO_GetLibRef(&ioLib, &error);
  if (error == aErrNone)
    aStem_GetLibRef(&stemLib, &error);

  if( error != aErrNone )
    ROS_WARN("Problem getting librefs\n");
}

void AcronameMotor::SetupPort(const string &portname, const long &portspeed) {
  /* Build a link stream to communicate serially with the stem. */
  if (error == aErrNone)
    aStream_CreateSerial(ioLib, 
                         portname.c_str(), 
                         portspeed,                  
                         &linkStream, 
                         &error);

  if( error != aErrNone ) {
    ROS_WARN("Problem forming link to Moto... ");
    
    switch(error) {
    case aErrNotFound:
      ROS_WARN("port not found!\n");
      break;
    case aErrRange:
      ROS_WARN("baud rate not available\n");
      break;
    case aErrOverrun:
      ROS_WARN("buffer overrun\n");
    default:
      ROS_WARN("unknown problem\n");
    };
  }

  if (error == aErrNone)
    aStem_SetStream(stemLib, linkStream, kStemModuleStream, &error);

}

void AcronameMotor::SetupChannel(const MotoAddr &addr, bool forward) {
  unsigned char mode_flags = 0;
  if (forward) {
    mode_flags |= (1 << aMOTION_PWMFLAG_INVPID);
  } else {
    mode_flags |= (1 << aMOTION_PWMFLAG_INVPWM);
  }

  error = aMotion_SetMode(stemLib, addr.module, addr.channel,
                          aMOTION_MODE_ENCVEL, mode_flags);

  if( error != aErrNone ) {
    ROS_WARN("Problem setting up channels on %s", addr.String().c_str());
  }
}

void AcronameMotor::
SetPWMFreq(const MotoAddr &addr, unsigned char paramH, unsigned char paramL) {
  // See acroname's site for details on parameters
  // http://www.acroname.com/brainstem/ref/ref.html#Commands/cmdMO_CFG.html

  // PWM period = [(4^paramH)*(paramL+1)*0.1us]
  // PWM freq = 1 / PWM period
  // PWM resolution = log(40MHz / PWM freq)/log(2)

  double period = pow(4, paramH) * (paramL + 1) * 1e-7;
  double freq = 1 / period;
  double resolution = log(40 * 10e6 / freq) / log(2);
  int val = paramL;
  val |= (paramH << 8);

  ROS_INFO("paramL: %d, paramH: %d, PWM Freq = %.2f Resolution = %0.1f",
           paramL, paramH, freq, resolution);
  
  error = aMotion_SetParam(stemLib, addr.module, addr.channel,
                           aMOTION_PARAM_PWMFREQ, val);

  if(error != aErrNone) {
    ROS_WARN("Problem setting PWMFreq on %s", addr.String().c_str());
  }
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
SetupPID(const MotoAddr &addr, double p, double i, double d, double period) {
  // Set motor PID params.  The resolution step size is fixed width, see
  // documentaiton for details
  std::string addr_str = addr.String();
  const char *addr_char = addr_str.c_str();
  error = aMotion_SetParam(stemLib, addr.module, addr.channel,
                           aMOTION_PARAM_P, fixed_width(p));
  if (error != aErrNone) { ROS_WARN("Error setting P on %s", addr_char); }

  error = aMotion_SetParam(stemLib, addr.module, addr.channel,
                           aMOTION_PARAM_I, fixed_width(i));
  if (error != aErrNone) { ROS_WARN("Error setting I on %s", addr_char); }

  error = aMotion_SetParam(stemLib, addr.module, addr.channel,
                           aMOTION_PARAM_D, fixed_width(d));
  if (error != aErrNone) { ROS_WARN("Error setting D on %s", addr_char); }

  /* Set the Period value. The increment value that can be stored is 
   * in 0.1msec increments. Depending on the Moto firmware, the lowest 
   * possible value is 1msec, which is a value of 10. 
   */
  error = aMotion_SetParam(stemLib, addr.module, addr.channel,
                           aMOTION_PARAM_PERIOD, (int)(round(period/0.0001)));
  if (error != aErrNone) { ROS_WARN("Error setting period on %s", addr_char); }

}

void AcronameMotor::SetVel(const MotoAddr &addr, short int vel) {
  error = aMotion_SetValue(stemLib, addr.module, addr.channel, vel);
  if (error != aErrNone) {
    ROS_WARN("Error setting velocity on %s", addr.String().c_str());
  }
}

void AcronameMotor::GetVel(const MotoAddr &addr, short int *vel) {
  error = aMotion_GetPIDInput(stemLib, addr.module, addr.channel, vel);
  if (error != aErrNone) {
    ROS_WARN("Error getting PID input on %s", addr.String().c_str());
  }
}

AcronameMotor::~AcronameMotor() {
  aStem_ReleaseLibRef(stemLib, NULL);
  aIO_ReleaseLibRef(ioLib, NULL);
}

bool AcronameMotor::ok() {
  return error == aErrNone;
}
