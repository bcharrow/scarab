/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStream_TextLine.c					   */
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

#include "aStream_TextLine.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * simple struct to hold text pointer
 */
 
typedef struct aTextLineData {
  char*		pLine;
  int		length;
  int		current;
} aTextLineData;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * static local routines (callbacks)
 */

static aErr sTextLine_Get(char* pData,
                          void* ref);
static aErr sTextLine_Delete(void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_Create_TextLine_Input
 */

aErr aStream_Create_TextLine_Input(aIOLib libRef, 
				   const char* line,
				   aStreamRef* pStreamRef)
{
  aErr ioErr;
  aStreamRef streamRef;
  aTextLineData* pTextLineData = NULL;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params
   */
  if (pStreamRef == NULL)
    return aErrParam;
  *pStreamRef = NULL;

  pTextLineData = (aTextLineData*)aMemAlloc(sizeof(aTextLineData));
  if (pTextLineData == NULL)
    return aErrMemory;

  aBZero(pTextLineData, sizeof(aTextLineData));

  pTextLineData->length = (int)aStringLen(line);
  pTextLineData->pLine = (char*)aMemAlloc((aMemSize)pTextLineData->length);

  /* string but no space for string is an error */
  if (pTextLineData->length && pTextLineData->pLine == NULL) {
    sTextLine_Delete(pTextLineData);
    return aErrMemory;
  }

  if (pTextLineData->pLine)
    aMemCopy(pTextLineData->pLine, line, (aMemSize)pTextLineData->length);

  if (aStream_Create(libRef, sTextLine_Get,
  		     NULL, sTextLine_Delete,
  		     pTextLineData, &streamRef, &ioErr)) {
    sTextLine_Delete(pTextLineData);
    return ioErr;
  }

  *pStreamRef = streamRef;
  
  return(ioErr);

} /* aStream_Create_TextLine_Input */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTextLine_Get
 */

aErr sTextLine_Get(char* pData,
		   void* ref)
{
  aTextLineData* pTextLineData = (aTextLineData*)ref;

  if (!pTextLineData)
    return aErrIO;

  /* returns EOF first time if string was empty */
  if (pTextLineData->current >= pTextLineData->length)
    return aErrEOF;

  *pData = pTextLineData->pLine[pTextLineData->current++];

  return(aErrNone);

} /* sTextLine_Get */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTextLine_Delete
 */

aErr sTextLine_Delete(void* ref)
{
  aTextLineData* pTextLineData = (aTextLineData*)ref;

  if (pTextLineData == NULL)
    return aErrIO;

  if (pTextLineData->pLine) {
    aMemFree((aMemPtr)pTextLineData->pLine);
    pTextLineData->pLine = NULL;
  }
  aMemFree((aMemPtr)pTextLineData);

  return aErrNone;

} /* sTextLine_Delete */
