/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTextDisplay.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*								   */
/* description: Provides a cross-platform text display object      */
/*              that manages variable sized windows and types of   */
/*              text.                                              */
/*								   */
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

#include "aTextDisplay.h"

typedef struct aTDLine {
  aTextDisplayType	type;
  unsigned int		bufferSize;
  unsigned int		length;
  unsigned int		displayLines;
  struct aTDLine*	pPrev;
  struct aTDLine*	pNext;
  char			buffer[1];
} aTDLine;

typedef struct aTD {
  unsigned int		nWidth;
  unsigned int		nMaxLines;
  unsigned int		nNumLines;
  unsigned int		nTotalDisplayLines;
  aTextDisplayType	computedType;
  aTDLine*		pTop;
  aTDLine*		pBottom;
  aTDLine*		pFree;
  aTDLine*		pCurLine;
  unsigned int		nCurrentDisplay;
  unsigned int 		nCurrentOffset;

  int			check;
} aTD;

#define aTDCHECK	0xF223

#define aVALIDTD(p)						   \
  if ((p == NULL) ||						   \
      (((aTD*)p)->check != aTDCHECK)) {		   		   \
    err = aErrParam;						   \
  }


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aTDLine* sGetLineStorage(aTD* pTD,
				const unsigned int minSize);
static void sFreeLineStorage(aTD* pTD,
			     aTDLine* pLine);
static void sRecomputeDisplay(aTD* pTD,
			      aTextDisplayType typeMask,
			      unsigned int nWidth);
static unsigned int sDisplayLines(const unsigned int nWidth,
			  	  const unsigned int nLength);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sGetLineStorage
 *
 * Gets the smallest available free line storage that will fit, If 
 * none will fit, it allocates a new one.  The free list just grows
 * and they are never released until cleanup.
 */

aTDLine* sGetLineStorage(
  aTD* pTD,
  const unsigned int minSize
)
{
  aTDLine* pLine = (aTDLine*)NULL;

  aAssert(pTD);
  
  if (pTD->pFree) {
    aTDLine* pPrev = (aTDLine*)NULL;
    aTDLine* pTemp = pTD->pFree;

    /* walk the list and find the smallest available free line */
    while (pTemp && (pTemp->bufferSize < minSize)) {
      pPrev = pTemp;
      pTemp = pTemp->pNext;
    }
   
    /* if a line was found, unlink it and return it */
    if (pTemp) {
      if (pPrev)
        pPrev->pNext = pTemp->pNext;
      else
        pTD->pFree = pTemp->pNext;
      pLine = pTemp;
      
    /* else, just allocate the requested size and return it */
    }
  }

  if (!pLine) {
    pLine = (aTDLine*)aMemAlloc((aMemSize)(sizeof(aTDLine) + minSize));
    if (pLine)
      pLine->bufferSize = minSize;
  }

  if (pLine) {
    pLine->length = minSize;
    pLine->pNext = (aTDLine*)NULL;
  }

  return pLine;

} /* sGetLineStorage */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sFreeLineStorage
 *
 * Puts the line storage buffer back onto the free list which is
 * sorted from smallest buffer to largest.
 */

void sFreeLineStorage(aTD* pTD, 
		      aTDLine* pLine)
{
  aTDLine* pTemp = (aTDLine*)NULL;
  aTDLine* pPrev = (aTDLine*)NULL;

  aAssert(pTD);

  pTemp = pTD->pFree;
  
  while (pTemp && (pLine->bufferSize > pTemp->bufferSize)) {
    pPrev = pTemp;
    pTemp = pTemp->pNext;
  }
  
  pLine->pNext = pTemp;
  if (pPrev)
    pPrev->pNext = pLine;
  else
    pTD->pFree = pLine;

} /* sFreeLineStorage */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRecomputeDisplay
 *
 * Computes the amount of display lines each actual line will take 
 * up.
 */

void sRecomputeDisplay(aTD* pTD,
		       aTextDisplayType typeMask,
		       unsigned int nWidth)
{
  aAssert(pTD);
  
  if ((typeMask != pTD->computedType) ||
      (nWidth != pTD->nWidth)) {
    aTDLine* pTemp = pTD->pTop;
    pTD->nTotalDisplayLines = 0;
    while (pTemp) {
      /* compute the amount of display lines handle this line */
      pTemp->displayLines = sDisplayLines(nWidth, pTemp->length);

      /* add this to the total display line count if visible */
      if (pTemp->type & typeMask)
        pTD->nTotalDisplayLines += pTemp->displayLines;
     
      /* get the next line */
      pTemp = pTemp->pNext;
    }
    pTD->computedType = typeMask;
    pTD->nWidth = nWidth;
  }

} /* sRecomputeDisplay */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sDisplayLines
 *
 * Computes the number of display lines needed to display a given
 * buffer line.
 */

unsigned int sDisplayLines(const unsigned int nWidth,
                           const unsigned int nLength)
{
  unsigned int left = nLength;
  unsigned int lines = 1;

  while (nWidth && (left >= nWidth)) {
    lines++;
    left -= (nWidth - 1);
  }
  
  return lines;

} /* sDisplayLines */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTextDisplay_Create
 */

aErr aTextDisplay_Create(const aTextDisplayType type,
			 const unsigned int numLines,
			 const unsigned int width,
			 aTextDisplayRef* pTDRef)
{
  aErr err = aErrNone;
  aTD* pTD = (aTD*)NULL;

  /* sanity check and initialize */
  if (!pTDRef)
    err = aErrParam;
  else
    *pTDRef = NULL;
  
  if (err == aErrNone) {
    pTD = (aTD*)aMemAlloc(sizeof(aTD));
    if (!pTD) {
      err = aErrMemory;
    } else {
      aBZero(pTD, sizeof(aTD));
      pTD->check = aTDCHECK;
      pTD->nMaxLines = numLines;
      pTD->nWidth = width;
      pTD->computedType = type;
      *pTDRef = pTD;
    }
  }

  return err;

} /* aTextDisplay_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTextDisplay_Destroy
 */

aErr aTextDisplay_Destroy(aTextDisplayRef tdRef)
{
  aErr err = aErrNone;
  aTD* pTD = (aTD*)tdRef;

  aVALIDTD(pTD);
  
  /* invalidate the magic cookie and destroy */
  if (err == aErrNone) {
    aTDLine* pTemp = pTD->pFree;
    aTDLine* pDead;
    while (pTemp) {
      pDead = pTemp;
      pTemp = pTemp->pNext;
      aMemFree((aMemPtr)pDead);
    }
    pTemp = pTD->pTop;
    while (pTemp) {
      pDead = pTemp;
      pTemp = pTemp->pNext;
      aMemFree((aMemPtr)pDead);
    }
    pTD->check = 0;
    aMemFree((aMemPtr)pTD);    
  }

  return err;

} /* aTextDisplay_Destroy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTextDisplay_SetWidth
 */

aErr aTextDisplay_SetWidth(aTextDisplayRef tdRef,
			   const unsigned int width)
{
  aErr err = aErrNone;
  aTD* pTD = (aTD*)tdRef;

  aVALIDTD(pTD);

  if (err == aErrNone) {
    sRecomputeDisplay(pTD, pTD->computedType, width);
  }

  return err;

} /* aTextDisplay_SetWidth */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTextDisplay_AddLine
 *
 * Adds the line to the bottom of the display list.  The lines are
 * listed in a singly-linked list from bottom to top.
 */

aErr aTextDisplay_AddLine(aTextDisplayRef tdRef,
			  const char* line,
			  const aTextDisplayType typeMask)
{
  aErr err = aErrNone;
  aTD* pTD = (aTD*)tdRef;

  aVALIDTD(pTD);

  if (err == aErrNone) {
    unsigned int len = (unsigned int)aStringLen(line);
    aTDLine* pLine;

    /* get the storage for the line */
    pLine = sGetLineStorage(pTD, len);

    /* compute the amount of display lines handle this line */
    pLine->displayLines = sDisplayLines(pTD->nWidth, len);

    /* add this to the total display line count */
    if (typeMask & pTD->computedType)
      pTD->nTotalDisplayLines += pLine->displayLines;

    /* copy in the attributes */
    aStringCopySafe(pLine->buffer, len + 1, line);
    pLine->type = typeMask;

    /* link it into the line list */
    if (!pTD->pTop)
      pTD->pTop = pLine;
    if (pTD->pBottom)
      pTD->pBottom->pNext = pLine;
    pLine->pPrev = pTD->pBottom;
    pTD->pBottom = pLine;

    /* if the buffer is full, roll off the top line */
    if (pTD->nMaxLines && (pTD->nNumLines >= pTD->nMaxLines)) {

      /* unlink it from the list */
      pLine = pTD->pTop;
      aAssert(pLine);
      pTD->pTop = pLine->pNext;
      pTD->pTop->pPrev = (aTDLine*)NULL;

      /* remove this from the total number of display lines */
      if (pLine->type & pTD->computedType)
        pTD->nTotalDisplayLines -= pLine->displayLines;

      /* free up the storage */
      sFreeLineStorage(pTD, pLine);

    /* otherwise, count it in the total current lines */
    } else {
      pTD->nNumLines++;
    }
  }

  return err;

} /* aTextDisplay_AddLine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTextDisplay_GetDisplayLines
 */

aErr aTextDisplay_GetDisplayLines(aTextDisplayRef tdRef,
			          unsigned int* pNumLines,
			          const aTextDisplayType typeMask)
{
  aErr err = aErrNone;
  aTD* pTD = (aTD*)tdRef;

  aVALIDTD(pTD);

  if ((err == aErrNone) && !pNumLines)
    err = aErrParam;

  if (err == aErrNone) {
    aAssert(pNumLines);
    if (typeMask != pTD->computedType)
      sRecomputeDisplay(pTD, typeMask, pTD->nWidth);
    *pNumLines = pTD->nTotalDisplayLines;
  }

  return err;

} /* aTextDisplay_GetDisplayLines */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTextDisplay_PrepareEnum
 *
 * Sets up the text display for displaying a set number of lines
 * from the bottom of the list.  This accounts for wrap-around lines
 * in the display.
 */

aErr aTextDisplay_PrepareEnum(aTextDisplayRef tdRef,
			      const unsigned int nStartLine,
			      const aTextDisplayType typeMask)
{
  aErr err = aErrNone;
  aTD* pTD = (aTD*)tdRef;

  aVALIDTD(pTD);

  if (err == aErrNone) {
    aTDLine* pTemp = pTD->pTop;
  
    /* make sure our computations are current */
    if (typeMask != pTD->computedType)
      sRecomputeDisplay(pTD, typeMask, pTD->nWidth);
 
    /* from the top, find the requested starting display line */
    pTD->nCurrentDisplay = 0;
    while (pTemp &&
    	   (pTemp->displayLines + pTD->nCurrentDisplay < nStartLine)) {
      pTD->nCurrentDisplay += pTemp->displayLines;
      pTemp = pTemp->pNext;
    }
    
    /* now we are on the correct logical line, see if we need to
     * advance within it to get the correct display line */
    if (pTemp) {
      pTD->nCurrentOffset = nStartLine - pTD->nCurrentDisplay;
      aAssert(pTD->nCurrentOffset <= pTemp->displayLines);
      pTD->pCurLine = pTemp;
    } else {
      pTD->pCurLine = (aTDLine*)NULL;
    }
  }

  return err;

} /* aTextDisplay_PrepareEnum */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTextDisplay_NextEnum
 */

aErr aTextDisplay_NextEnum(aTextDisplayRef tdRef,
			   aStreamRef buffer,
			   const unsigned int maxLen,
			   aTextDisplayType* pTypeMask)
{
  aErr err = aErrNone;
  aTD* pTD = (aTD*)tdRef;

  aVALIDTD(pTD);

  /* start with a clean line */
  if (err == aErrNone)
    aStream_Flush(aStreamLibRef(buffer), buffer, NULL, &err);

  /* now, see if data is available */
  if (err == aErrNone) {

    if (pTD->pCurLine) {

      /* if we are at the end of a logical line, advance to 
       * the next */
      if (pTD->nCurrentOffset >= pTD->pCurLine->displayLines) {
        pTD->pCurLine = pTD->pCurLine->pNext;
        pTD->nCurrentOffset = 0;
      }

      /* if we now have a line, copy it into the provided buffer */
      if (pTD->pCurLine) {
        char *s = &pTD->pCurLine->buffer[pTD->nCurrentOffset * (pTD->nWidth - 1)];
        unsigned int len = (unsigned int)aStringLen(s);
        
        if (len < (pTD->nWidth - 1)) {
          aStream_Write(aStreamLibRef(buffer), buffer,
          		s, len + 1, &err);
        } else {
          aStream_Write(aStreamLibRef(buffer), buffer,
          		s, pTD->nWidth - 1, &err);
          if (err == aErrNone)
            aStream_Write(aStreamLibRef(buffer), buffer,
          		  "\\\0", 2, &err);
        }

	/* store the type if requested */
        if (pTypeMask)
          *pTypeMask = pTD->pCurLine->type;

        pTD->nCurrentOffset++;
      }
    }
  }

  return err;

} /* aTextDisplay_NextEnum */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTextDisplay_GetLine
 *
 * Tries to get a line of the specified type indexed from the 
 * bottom and returns a range error if it cannot be found.
 */

aErr aTextDisplay_GetLine(aTextDisplayRef tdRef,
			  const unsigned int nIndex,
			  const aTextDisplayType typeMask,
			  aStreamRef buffer)
{
  aErr err = aErrNone;
  aTD* pTD = (aTD*)tdRef;

  aVALIDTD(pTD);
  
  if (err == aErrNone) {
    aTDLine* pLine = pTD->pBottom;
    unsigned int i = 0;
  
    while (pLine) {
      if (pLine->type & typeMask) {
        if (i == nIndex)
          break;
        i++;
      }
      pLine = pLine->pPrev;
    }

    if (pLine) {
      unsigned int len = (unsigned int)aStringLen(pLine->buffer);
      /* first, clear out the buffer */
      aStream_Flush(aStreamLibRef(buffer), buffer, 
      			  NULL, &err);
      if (err == aErrNone)
        aStream_Write(aStreamLibRef(buffer), buffer,
      		      pLine->buffer, len + 1, &err);
    } else {
      err = aErrRange;
    }
  }

  return err;

} /* aTextDisplay_GetLine */
