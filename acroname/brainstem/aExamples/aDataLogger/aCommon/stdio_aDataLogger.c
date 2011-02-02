/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: stdio_aDataLogger.c                                       */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: The main routine of the stdio version of the       */
/*              stdio aDataLogger tutorial application.            */
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

#include "aStream_STDIO_Console.h"
#include "aDataLogger.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * main
 */

int main(int argc, char* argv[])
{
  aErr err = aErrNone;
  aIOLib ioRef;
  aStreamRef output;
  int retVal = 0;

  /* get the aIO library reference */
  aIO_GetLibRef(&ioRef, &err);

  /* create a stream on standard output */
  if (err == aErrNone)
    err = aStream_Create_STDIO_Console_Output(ioRef, &output);

  /* run the program */
  if (err == aErrNone)
    retVal = aDataLogger_Execute(output);

  /* release the aIO library */
  if (err == aErrNone)
    aIO_ReleaseLibRef(ioRef, &err);

  if (err)
    return err;
  else
    return retVal;

} /* main */
