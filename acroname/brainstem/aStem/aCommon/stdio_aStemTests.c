/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: stdio_aStemTests.c                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: stdio testing module for the aStem shared library. */
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

#include "aMemLeakDebug.h"

#include "aStemTests.h"
#include "aStream_STDIO_Console.h"
#include "aMemLeakDebug.h"

#ifdef aWINCE
  int aIO_StartTests() {
#else
  int main(int argc, char* argv[]) {
#endif

  aErr err;
  aIOLib ioRef = NULL;
  aStreamRef output = NULL;

  if (aIO_GetLibRef(&ioRef, &err))
    return 1;

  err = aStream_Create_STDIO_Console_Output(ioRef, &output);
  if (err != aErrNone)
    return 1;

  aStream_WriteLine(ioRef, output, "STDIO Stem Tests Starting", NULL);

#if 0
  if (aStemTests(ioRef, output))
    aStream_WriteLine(ioRef, output, "!!! FAILED !!!", NULL);

  else 
  if (aStemLiveTests(ioRef, output))
    aStream_WriteLine(ioRef, output, "!!! FAILED !!!", NULL);

  else
#endif
  if (aStemRelayTests(ioRef, output))
    aStream_WriteLine(ioRef, output, "!!! FAILED !!!", NULL);
    
  aStream_WriteLine(ioRef, output, "STDIO Stem Tests Finished", NULL);

  aStream_Destroy(ioRef, output, NULL);

  aIO_ReleaseLibRef(ioRef, NULL);

  aLeakCheckCleanup();

  return 0;

} /* main */

