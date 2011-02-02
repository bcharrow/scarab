/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aStub.c	     	                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Simple "Hello World" type application for the	   */
/*		BrainStem using Unix.	   			   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Copyright 1994-2008. Acroname Inc.                              */
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

#include "aStem.h"	/* stem library header   */
#include "aServo.h"	/* servo control routines */
#include "aDigital.h"   /* digital I/O routines */
#include "aAnalog.h"

/* define which BrainStem module and servo we are talking to and 
 * how. In this case, we assume a GP 1.0 module */
#define aMODULE	2
#define aSERVO 0
#define aPORTNAME "ttyS0"
#define aPORTSPEED 9600

/* the main (and only) routine */
int main(int argc, char* argv[]) {
  aErr stubErr = aErrNone;
  int i;
  aIOLib ioLib;
  aStemLib stemLib;
  aStreamRef linkStream;
  char cBytes[4];
  unsigned char cValue;
  int iValue;

  /* Get the references to the aIO and aStem library objects. */
  if (stubErr == aErrNone)
    aIO_GetLibRef(&ioLib, &stubErr);
  if (stubErr == aErrNone)
    aStem_GetLibRef(&stemLib, &stubErr);

  /* Build a link stream to communicate serially with the stem. */
  if (stubErr == aErrNone)
    aStream_CreateSerial(ioLib, aPORTNAME, aPORTSPEED, 
    			 &linkStream, &stubErr);

  /* Set this new stream as the stem's link stream. This stream
   * will automatically be destroyed when the stem is destroyed. */
  if (stubErr == aErrNone)
    aStem_SetStream(stemLib, linkStream, kStemModuleStream, &stubErr);



  /*****************************/
  /* Test some servo functions */
  /*****************************/

  /* wiggle the servo with 90 degree range */
  cBytes[0] = 0x11;
  cBytes[1] = 0x23;
  stubErr = aServo_SetLimits(stemLib, aMODULE, aSERVO, 
			     (unsigned char*)cBytes);
  stubErr = aServo_GetLimits(stemLib, aMODULE, aSERVO, 
			     (unsigned char*)cBytes);
  printf("limits = 0x%02X,0x%02X\n", cBytes[0], cBytes[1]);
  for (i = 0; (i < 5) && (stubErr == aErrNone); i++) {

      /* display where we are */
      printf("90 degree range, pass %d\n", i);

      stubErr = aServo_SetPositionAbs(stemLib, aMODULE, aSERVO, 0);
      aIO_MSSleep(ioLib, 400, NULL);
      stubErr = aServo_GetPosition(stemLib, aMODULE, aSERVO, &cValue);

      stubErr = aServo_SetPositionAbs(stemLib, aMODULE, aSERVO, 255);      
      aIO_MSSleep(ioLib, 600, NULL);
      stubErr = aServo_GetPosition(stemLib, aMODULE, aSERVO, &cValue);
  }
  
  /* invert servo */
  printf("invert...\n");
  stubErr = aServo_GetConfig(stemLib, aMODULE, aSERVO, &cValue);      
  stubErr = aServo_SetConfig(stemLib, aMODULE, aSERVO, (unsigned char)(cValue | aSERVO_INV));      
  aIO_MSSleep(ioLib, 2000, NULL);

  /* wiggle the servo with 180 degree range */
  cBytes[0] = 0x00;
  cBytes[1] = 0x46;
  stubErr = aServo_SetLimits(stemLib, aMODULE, aSERVO, 
			     (unsigned char*)cBytes);
  stubErr = aServo_GetLimits(stemLib, aMODULE, aSERVO, 
			     (unsigned char*)cBytes);
  printf("limits = 0x%02X,0x%02X\n", cBytes[0], cBytes[1]);
  for (i = 0; (i < 5) && (stubErr == aErrNone); i++) {

      /* display where we are */
      printf("180 degree range, pass %d\n", i);

      stubErr = aServo_SetPositionAbs(stemLib, aMODULE, aSERVO, 0);
      aIO_MSSleep(ioLib, 700, NULL);
      stubErr = aServo_GetPosition(stemLib, aMODULE, aSERVO, &cValue);

      stubErr = aServo_SetPositionAbs(stemLib, aMODULE, aSERVO, 255);      
      aIO_MSSleep(ioLib, 900, NULL);
      stubErr = aServo_GetPosition(stemLib, aMODULE, aSERVO, &cValue);
  }

  /* un-invert servo */
  printf("un-invert...\n");
  stubErr = aServo_GetConfig(stemLib, aMODULE, aSERVO, &cValue);      
  stubErr = aServo_SetConfig(stemLib, aMODULE, aSERVO, (unsigned char)(cValue & ~aSERVO_INV));      
  aIO_MSSleep(ioLib, 2000, NULL);



  /******************************/
  /* Now test some IO functions */
  /******************************/

  /* read A2D 0, read digital IO 0, and flash IO 1 */
  if (stubErr == aErrNone) {
    stubErr = aDigital_SetConfig(stemLib, aMODULE, 1, aDIGITAL_OUTPUT);
    for (i = 0; (i<100) && (stubErr == aErrNone); i++) {
      stubErr = aAnalog_ReadInt(stemLib, aMODULE, 0, &iValue);
      printf("A2D0 = %i, ", iValue);
      stubErr = aDigital_ReadInt(stemLib, aMODULE, 0, &iValue);
      printf("DIG0 = %i\n", iValue);
      stubErr = aDigital_WriteInt(stemLib, aMODULE, 1, i % 2);
      aIO_MSSleep(ioLib, 500, NULL);
    }
  }



  /* release the libraries now that we are done whether there
   * were errors or not */

  aStem_ReleaseLibRef(stemLib, NULL);
  aIO_ReleaseLibRef(ioLib, NULL);

  return 0;

} /* main */
