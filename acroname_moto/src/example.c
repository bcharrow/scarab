/*                                                                 */
/* file: xxxxxxxx.c                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Simple program to demonstrate using the BrainStem  */
/* Moto using the C libraries.                                 */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Copyright 1994-2007. Acroname Inc.                              */
/*                                                                 */
/* This software is the property of Acroname Inc.  Any             */
/* distribution, sale, transmission, or re-use of this code is     */
/* strictly forbidden except with permission from Acroname Inc.    */
/*                                                                 */
/* To the full extent allowed by law, Acroname Inc. also excludes  */
/* for itself and its suppliers any liability, wheither based in   */
/* contract or tort (including negligence), for direct,            */
/* incidental, consequential, indirect, special, or punitive       */
/* damages of any kind, or for loss of revenue or profits, loss of */
/* business, loss of information or data, or other financial loss  */
/* arising out of or in connection with this software, even if     */
/* Acroname Inc. has been advised of the possibility of such       */
/* damages.                                                        */
/*                                                                 */
/* Acroname Inc.                                                   */
/* www.acroname.com                                                */
/* 720-564-0373                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <aCommon/aStem.h>    /* stem library header   */
#include <aCommon/aMotion.h> /* motion control library */
#include <stdlib.h>

/* define which BrainStem module and servo we are talking to and 
 * how. In this case, we assume a GP 1.0 module 
 */
#define aMODULE    4
#define aCHANNEL 0

/* the portname below will depend entirely on your computer&#39;s      */
/* serial port configuration.  We use the command                  */
/*   ls /dev/tty.*                                                 */
/* to learn what the name of the serial adapter is on our machines */
/* Note: This value will change if you plug the adapter in other   */
/* USB ports, depending on your adapter type.                      */
#define aPORTNAME "acroname"
#define aPORTSPEED 9600

/* Program specific macros */
#define LOOPDELAY 250
#define SETMAX 200
#define SETINC 10

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* the main (and only) routine */
int main(int argc, char* argv[]) {
  aErr error = aErrNone;
  aIOLib ioLib;
  aStemLib stemLib;
  aStreamRef linkStream;
  short setpoint = 0;
  short pidval = 0;
  
  printf("Running the motor example.\n");     
  
  /* Get the references to the aIO and aStem library objects. */
  if (error == aErrNone)
    aIO_GetLibRef(&ioLib, &error);
  if (error == aErrNone)
    aStem_GetLibRef(&stemLib, &error);

  if( error != aErrNone )
    printf("Problem getting librefs\n");

  
  /* Build a link stream to communicate serially with the stem. */
  if (error == aErrNone)
    aStream_CreateSerial(ioLib, 
                         aPORTNAME, 
                         aPORTSPEED,                  
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
  
  /* Set this new stream as the stem&#39;s link stream. This stream
   * will automatically be destroyed when the stem is destroyed. 
   */
  if (error == aErrNone)
    aStem_SetStream(stemLib, 
                    linkStream, 
                    kStemModuleStream, 
                    &error);
  
  /* Configure the Moto settings
   * These steps can be skipped if you want to manually change the 
   * settings of the Moto through the Moto Application and save the 
   * settings to the EEPROM. The next few commands only demonstrate 
   * ways to manipulate the values using the C libraries.
   * 
   * See the aMotionDef.tea file found in the aSystem folder for a 
   * list of macros that can be used to define most setting values.
   *
   * If there was an error earlier, then we shouldn&#39;t even bother
   * trying to change the settings.
   */

  if (error == aErrNone)


    {                

      unsigned char mode = 0;
      unsigned char mode_flags = 0;

      mode_flags |= (1 << aMOTION_PWMFLAG_INVPID);

      /* We want to try out the pwm only mode */
      error = aMotion_SetMode(stemLib,
                              aMODULE,
                              aCHANNEL,
                              aMOTION_MODE_ENCVEL,
                              mode_flags);

    
      /* Set the PID P term. The values for this are stored on the Moto 
       * in fixed point notations. The resolution step size for the term 
       * is in increments of 0.031. Therefore, setting the value to 32 
       * would set the term to 0.031 X 32 = 0.99199 (aka 1.00).
       */
      error = aMotion_SetParam(stemLib,
                               aMODULE,
                               aCHANNEL,
                               aMOTION_PARAM_P,
                               32);

      /* Set the PID I term. See the note above about the P term. */ 
      error = aMotion_SetParam(stemLib,
                               aMODULE,
                               aCHANNEL,
                               aMOTION_PARAM_I,
                               0);
    
      /* Set the Period value. The increment value that can be stored is 
       * in 0.1msec increments. Depending on the Moto firmware, the lowest 
       * possible value is 1msec, which is a value of 10. 
       */
      error = aMotion_SetParam(stemLib,
                               aMODULE,
                               aCHANNEL,
                               aMOTION_PARAM_PERIOD,
                               200);



    
    } // End of changing the Moto settings

  
  /* Begin spinning the motor.
   *
   * If there was an error earlier getting started, then there is no 
   * point in trying to spin a motor. 
   */
  if (error == aErrNone)
    {
    
      /* Slowly increment the setpoint */
      for (setpoint = 0; setpoint < SETMAX; setpoint = setpoint + SETINC)
        {
          printf("Setting the Setpoint value to %d\n",setpoint);
          error = aMotion_SetValue(stemLib,
                                   aMODULE,
                                   aCHANNEL,
                                   setpoint);
          /* Check to see if the setpoint is zero. We don&#39;t want to try and 
           * chase this as a PID error. The motor should be stopped.
           */
          if (setpoint > 0)
            {                           
              do {
      
                error = aMotion_GetPIDInput(stemLib,
                                            aMODULE,
                                            aCHANNEL,
                                            &pidval);
      
                printf("\tPID Return val is: %d Setpoint: %d\n",
                       pidval,
                       abs((pidval - setpoint)));
      
              } while (abs((pidval - setpoint)) > 2);
            }
      
          /* Delay a little bit */
          aIO_MSSleep(ioLib, LOOPDELAY, NULL);
        }
    
      /* Slowly bring the setpoint back down to a stop */
      for (setpoint = setpoint; setpoint >= 0; setpoint = setpoint - SETINC)
        {
          printf("Setting the Setpoint value to %d\n",setpoint);     
          error = aMotion_SetValue(stemLib,
                                   aMODULE,
                                   aCHANNEL,
                                   setpoint);
          /* Check to see if the setpoint is zero. We don&#39;t want to try and 
           * chase this as a PID error. The motor should be stopped.
           */      
          if (setpoint > 0)
            {                           
              do {            
                error = aMotion_GetPIDInput(stemLib,
                                            aMODULE,
                                            aCHANNEL,
                                            &pidval);
      
                printf("\tPID Return val is: %d Setpoint: %d\n",
                       pidval,
                       abs((pidval - setpoint)));
      
              } while (abs((pidval - setpoint)) > 2);
            }                                       
      
          /* Delay a little bit */
          aIO_MSSleep(ioLib, LOOPDELAY, NULL);
        }
    }
  
  /* release the libraries now that we are done whether there
   * were errors or not 
   */
  aStem_ReleaseLibRef(stemLib, NULL);
  aIO_ReleaseLibRef(ioLib, NULL);
  
  return error;
  
} /* main */
