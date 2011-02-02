/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aParseFrame.c                                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: cross-platform parsing frame definition.           */
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

#include "aOSDefs.h"
#include "aParseFrame.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aParseFrame_Create
 */

aBool aParseFrame_Create(const char* frameName,
			 aStreamRef inputStream,
			 aParseFrame** ppFrame,
			 aErr* pErr)
{
  aErr err = aErrNone;
  aParseFrame* pFrame;

  /* check params */
  if ((ppFrame == NULL) || 
      (frameName == NULL) || 
      (inputStream == NULL))
    err = aErrParam;
  
  if (err == aErrNone) {
    pFrame = (aParseFrame*)aMemAlloc(sizeof(aParseFrame));
    if (pFrame == NULL)
      err = aErrMemory;
    else {
      aBZero(pFrame, sizeof(aParseFrame));
      pFrame->lineNum = 0;
      pFrame->columnNum = 1;
      aStringCopySafe(pFrame->frameName, aFILE_NAMEMAXCHARS, frameName);
      pFrame->input = inputStream;
      pFrame->bBuffered = aFalse;
      pFrame->bLineStart = aTrue;
      if (err == aErrNone)
        *ppFrame = pFrame;
    }
  }

  if (pErr != NULL)
    *pErr = err;

  return(err != aErrNone);

} /* aParseFrame_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aParseFrame_AddChild
 */

aBool aParseFrame_AddChild(aParseFrame* pParentFrame,
			   aParseFrame* pChildFrame,
			   aErr* pErr)
{
  aErr err = aErrNone;

  if (err == aErrNone) {
    if (pParentFrame->pChildren == NULL) {
      pParentFrame->pChildren = pChildFrame;
    } else {
      aParseFrame* pTemp = pParentFrame->pChildren;
      while (pTemp->pSiblings != NULL)
        pTemp = pTemp->pSiblings;
      pTemp->pSiblings = pChildFrame;
    }
    pChildFrame->pParent = pParentFrame;
  }

  return err;

} /* aParseFrame_AddChild */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aParseFrame_Destroy
 */

aBool aParseFrame_Destroy(aParseFrame* pFrame,
			  aErr* pErr)
{
  aErr err = aErrNone;

  /* check params */
  if (pFrame == NULL)
    err = aErrParam;

  /* recurse and clean up */
  if (pFrame->pChildren != NULL) {
    aParseFrame_Destroy(pFrame->pChildren, &err);
    pFrame->pChildren = NULL;
  }
  if (pFrame->pSiblings != NULL) {
    aParseFrame_Destroy(pFrame->pSiblings, &err);
    pFrame->pSiblings = NULL;
  }
  
  if (pFrame->input != NULL) {
    aStream_Destroy(aStreamLibRef(pFrame->input), 
        		          pFrame->input, &err);
    pFrame->input = NULL;
  }

  if (err == aErrNone)
    aMemFree((aMemPtr)pFrame);
  
  if (pErr != NULL)
    *pErr = err;

  return(err != aErrNone);

} /* aParseFrame_Destroy */
