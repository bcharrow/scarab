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
    aStem_SetStream(stemLib, 
                    linkStream, 
                    kStemModuleStream, 
                    &error);

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

void AcronameMotor::
SetupPID(double p, double i, double d, double period)
{
  /* Set the PID P term. The values for this are stored on the Moto 
   * in fixed point notations. The resolution step size for the term 
   * is in increments of 0.031. Therefore, setting the value to 32 
   * would set the term to 0.031 X 32 = 0.99199 (aka 1.00).
   */  
  error = aMotion_SetParam(stemLib,
                           aMODULE,
                           left_channel,
                           aMOTION_PARAM_P,
                           (int)(round(p/0.031)));

  error = aMotion_SetParam(stemLib,
                           aMODULE,
                           left_channel,
                           aMOTION_PARAM_I,
                           (int)(round(i/0.031)));

  error = aMotion_SetParam(stemLib,
                           aMODULE,
                           left_channel,
                           aMOTION_PARAM_D,
                           (int)(round(d/0.031)));

  error = aMotion_SetParam(stemLib,
                           aMODULE,
                           right_channel,
                           aMOTION_PARAM_P,
                           (int)(round(p/0.031)));

  error = aMotion_SetParam(stemLib,
                           aMODULE,
                           right_channel,
                           aMOTION_PARAM_I,
                           (int)(round(i/0.031)));

  error = aMotion_SetParam(stemLib,
                           aMODULE,
                           right_channel,
                           aMOTION_PARAM_D,
                           (int)(round(d/0.031)));

  /* Set the Period value. The increment value that can be stored is 
   * in 0.1msec increments. Depending on the Moto firmware, the lowest 
   * possible value is 1msec, which is a value of 10. 
   */
  error = aMotion_SetParam(stemLib,
                           aMODULE,
                           left_channel,
                           aMOTION_PARAM_PERIOD,
                           (int)(round(period/0.0001)));

  error = aMotion_SetParam(stemLib,
                           aMODULE,
                           right_channel,
                           aMOTION_PARAM_PERIOD,
                           (int)(round(period/0.0001)));
}

int AcronameMotor::
SetVel(short int left_vel, short int right_vel)
{
  //if(error == aErrNone) {
    error = aMotion_SetValue(stemLib,
                             aMODULE,
                             left_channel,
                             left_vel); 

    error = aMotion_SetValue(stemLib,
                             aMODULE,
                             right_channel,
                             right_vel);
  //}

  if(error == aErrNone)
    return 0;

  return -1;
}

int AcronameMotor::
GetVel(short int &left_vel, short int &right_vel)
{
  //if(error == aErrNone) {

    error = aMotion_GetPIDInput(stemLib,
                                aMODULE,
                                left_channel,
                                &left_vel);

    error = aMotion_GetPIDInput(stemLib,
                                aMODULE,
                                right_channel,
                                &right_vel);
  //}

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
