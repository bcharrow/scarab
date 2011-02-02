/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aPPRKStub.c	                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Testing module for the aPPRK shared library.	   */
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


#include "aPPRKStub.h"

#define nRANGES	50

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRKTests
 * 
 * returns true on error
 */

aBool aPPRKTests(aIOLib ioRef, aStreamRef out)
{
  aPPRKLib pprkRef;
  aErr pprkErr = aErrNone;
  char msg[200];
  int i;
  float value;
  aBool bFailed = aTrue;

  aStringCopy(msg, " Getting Library Reference...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (aPPRK_GetLibRef(&pprkRef, &pprkErr))
    goto cleanup;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* servo 0 test */
  aStringCopy(msg, "  Moving Servo 0...");
  aStream_WriteLine(ioRef, out, msg, NULL);
  if (aPPRK_SetServoAbs(pprkRef, 0, 0.3f, NULL))
    goto cleanup;
  if (aPPRK_Sleep(pprkRef, 2000, NULL))
    goto cleanup;
  if (aPPRK_SetServoAbs(pprkRef, 0, -0.3f, NULL))
    goto cleanup;
  if (aPPRK_Sleep(pprkRef, 2000, NULL))
    goto cleanup;
  if (aPPRK_SetServoAbs(pprkRef, 0, 0, NULL))
    goto cleanup;
  if (aPPRK_Sleep(pprkRef, 200, NULL))
    goto cleanup;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* servo 1 test */
  aStringCopy(msg, "  Moving Servo 1...");
  aStream_WriteLine(ioRef, out, msg, NULL);
  if (aPPRK_SetServoAbs(pprkRef, 1, 0.3f, NULL))
    goto cleanup;
  if (aPPRK_Sleep(pprkRef, 2000, NULL))
    goto cleanup;
  if (aPPRK_SetServoAbs(pprkRef, 1, -0.3f, NULL))
    goto cleanup;
  if (aPPRK_Sleep(pprkRef, 2000, NULL))
    goto cleanup;
  if (aPPRK_SetServoAbs(pprkRef, 1, 0, NULL))
    goto cleanup;
  if (aPPRK_Sleep(pprkRef, 200, NULL))
    goto cleanup;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* servo 2 test */
  aStringCopy(msg, "  Moving Servo 2...");
  aStream_WriteLine(ioRef, out, msg, NULL);
  if (aPPRK_SetServoAbs(pprkRef, 2, 0.3f, NULL))
    goto cleanup;
  if (aPPRK_Sleep(pprkRef, 2000, NULL))
    goto cleanup;
  if (aPPRK_SetServoAbs(pprkRef, 2, -0.3f, NULL))
    goto cleanup;
  if (aPPRK_Sleep(pprkRef, 2000, NULL))
    goto cleanup;
  if (aPPRK_SetServoAbs(pprkRef, 2, 0, NULL))
    goto cleanup;
  if (aPPRK_Sleep(pprkRef, 200, NULL))
    goto cleanup;
  aStream_WriteLine(ioRef, out, "passed", NULL);


  /* testing output of ranger 0 */
  aStringCopy(msg, "  Reading Ranger 0...");
  aStream_WriteLine(ioRef, out, msg, NULL);
  for (i = 1; i < nRANGES; i += 1) {
    if (aPPRK_GetRange(pprkRef, 0, &value, &pprkErr))
      goto cleanup;
    sprintf(msg, "  ranger 0 = %f", value);
    aStream_WriteLine(ioRef, out, msg, NULL);
    if (aPPRK_Sleep(pprkRef, 100, NULL))
      goto cleanup;
  }
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* testing output of ranger 1 */
  aStringCopy(msg, "  Reading Ranger 1...");
  aStream_WriteLine(ioRef, out, msg, NULL);
  for (i = 1; i < nRANGES; i += 1) {
    if (aPPRK_GetRange(pprkRef, 1, &value, &pprkErr))
      goto cleanup;
    sprintf(msg, "  ranger 1 = %f", value);
    aStream_WriteLine(ioRef, out, msg, NULL);
    if (aPPRK_Sleep(pprkRef, 100, NULL))
      goto cleanup;
  }
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* testing output of ranger 2 */
  aStringCopy(msg, "  Reading Ranger 2...");
  aStream_WriteLine(ioRef, out, msg, NULL);
  for (i = 1; i < nRANGES; i += 1) {
    if (aPPRK_GetRange(pprkRef, 2, &value, &pprkErr))
      goto cleanup;
    sprintf(msg, "  ranger 2 = %f", value);
    aStream_WriteLine(ioRef, out, msg, NULL);
    if (aPPRK_Sleep(pprkRef, 100, NULL))
      goto cleanup;
  }
  aStream_WriteLine(ioRef, out, "passed", NULL);

  bFailed = aFalse;

cleanup:
  aStringCopy(msg, " Releasing Library Reference...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (aPPRK_ReleaseLibRef(pprkRef, &pprkErr))
    return aTrue; 
  aStream_WriteLine(ioRef, out, "passed", NULL);

  return bFailed;

} /* aPPRKTests */
