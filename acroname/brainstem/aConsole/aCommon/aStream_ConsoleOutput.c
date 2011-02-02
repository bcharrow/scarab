/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStream_ConsoleOutput.c				   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: cross-platform text line stream			   */
/*              implementation.                                    */
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

#include "aConsole.h"
#include "aStream_ConsoleOutput.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * simple struct to hold text pointer
 */
 
typedef struct aCBD {
  aConsole*		pConsole;
  aStreamRef		outputBuffer;
  aTextDisplayType	type;
} aCBD;



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * static local routines (callbacks)
 */

static aErr sConsoleOutput_Put(char* pData,
                               void* ref);
static aErr sConsoleOutput_Delete(void* ref);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_CreateConsoleOutput
 */

aErr aStream_CreateConsoleOutput(aIOLib libRef, 
				 void* vpConsole,
				 aStreamRef* pStreamRef,
				 const aTextDisplayType type)
{
  aErr ioErr;
  aStreamRef streamRef;
  aCBD* pCBD;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params
   */
  if (pStreamRef == NULL)
    return aErrParam;
  *pStreamRef = NULL;

  pCBD = (aCBD*)aMemAlloc(sizeof(aCBD));
  if (pCBD == NULL)
    return aErrMemory;
 
  pCBD->pConsole = (aConsole*)vpConsole;
  if (aStreamBuffer_Create(libRef, 32, 
  		           &pCBD->outputBuffer,
  		           &ioErr))
    return ioErr;

  pCBD->type = type;

  if (aStream_Create(libRef, 
  		     NULL,
  		     sConsoleOutput_Put,
  		     sConsoleOutput_Delete,
  		     pCBD,
  		     &streamRef, 
  		     &ioErr))
    return ioErr;

  *pStreamRef = streamRef;
  
  return ioErr;

} /* aStream_CreateConsoleOutput */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsoleOutput_Put
 */

aErr sConsoleOutput_Put(char* pData,
			void* ref)
{
  aErr ioErr = aErrNone;
  aCBD* pCBD = (aCBD*)ref;

  if (!pCBD)
    return ioErr = aErrIO;

#ifdef aWIN
  /* just toss \r */
  if (*pData == '\r')
    return aErrNone;
#endif /* aWIN */

  /* write to screen on newline */
  if (ioErr == aErrNone) {
    aIOLib ioRef = pCBD->pConsole->ioLib;
    aAssert(ioRef);
  
    /* newlines dump to the display */
    if (*pData == '\n') {
      char* pLine;

      /* null terminate the line */
      aStream_Write(ioRef, pCBD->outputBuffer, "\0", 1, &ioErr);

      /* get the buffer */
      if (ioErr == aErrNone)
        aStreamBuffer_Get(ioRef, pCBD->outputBuffer, 
        		  NULL, &pLine, &ioErr);

      if (ioErr == aErrNone)
        ioErr = aConsole_DisplayLine(pCBD->pConsole, 
        			     pLine, pCBD->type);

      /* empty the buffer */
      if (ioErr == aErrNone)
        aStream_Flush(ioRef, pCBD->outputBuffer, 
        		    NULL, &ioErr);
    
    /* just accumulate all else */
    } else {
      aStream_Write(ioRef, pCBD->outputBuffer, pData, 1, NULL);
    }
  }

  return ioErr;

} /* sConsoleOutput_Put */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsoleOutput_Delete
 */

aErr sConsoleOutput_Delete(void* ref)
{
  aCBD* pCBD = (aCBD*)ref;

  if (pCBD == NULL)
    return aErrIO;
  
  if (pCBD->outputBuffer) {
    aStream_Destroy(pCBD->pConsole->ioLib, pCBD->outputBuffer, NULL);
    pCBD->outputBuffer = NULL;
  }

  aMemFree((aMemPtr)pCBD);

  return aErrNone;

} /* sConsoleOutput_Delete */
