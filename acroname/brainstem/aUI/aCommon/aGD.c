/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aGD.c	                                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent graphics  */
/*		layer.						   */
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

#include "aUIInternal.h"
#include "aGDOffscreen.h"
#include "aGD.h"

#include <math.h>


#define aBEZIERARCCONTROLRADIUS  .447715


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static void sGetP(
  aGD* pGD, 
  aGDP** ppGDP);
static void sAddP(
  aGD* pGD, 
  aGDP* pGDP);
static void sCleanP(
  aGD* pGD);
static aErr sCleanupLine(
  aGDP* pLinePrimitive);
static aErr sCleanupText(
  aGDP* pTextPrimitive);
static aErr sCleanupCopy(
  aGDP* pTextPrimitive);
static aErr sSetFont(
  aUI* pUI, 
  void* vpData,
  aGDP* pTextPrimitive);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sGetP
 *
 * allocates a aGDP struct 
 */

void sGetP(
  aGD* pGD,
  aGDP** ppGDP)
{
  aErr err;
  aMemPool_Alloc(pGD->pUI->ioRef, 
  		 (aMemPtr)pGD->pUI->pGDPPool, (void**)ppGDP, &err);
  aAssert(err == aErrNone);

  /* clean up the allocated struct */
  aBZero(*ppGDP, sizeof(aGDP));
  
  (*ppGDP)->pGD = pGD;

} /* sGetP */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sAddP
 *
 * appends an aGDP struct to the draw list 
 */

void sAddP(
  aGD* pGD,
  aGDP* pGDP)
{
  /* if the list is empty */
  if (pGD->pDrawlist)
    pGD->pDrawlast->pNext = pGDP;
  else
    pGD->pDrawlist = pGDP;

  /* we are always appending */
  pGD->pDrawlast = pGDP;

  aAssert(pGDP->pNext != pGDP);

} /* sAddP */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCleanP
 *
 * cleans up a GD draw list
 */

void sCleanP(
  aGD* pGD)
{
  /* step through and empty the list */
  while (pGD->pDrawlist) {
    aGDP* pTemp = pGD->pDrawlist;
    aAssert(pTemp != pTemp->pNext);
    pGD->pDrawlist = pTemp->pNext;
    if (pTemp->pCleanup)
      pTemp->pCleanup(pTemp);
    aMemPool_Free(pGD->pUI->ioRef, pGD->pUI->pGDPPool, 
    		  pTemp, (aErr*)NULL);
  }

} /* sCleanP */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCleanupLine
 */

aErr sCleanupLine(
  aGDP* pLinePrimitive)
{
  aErr err = aErrNone;

  aAssert(pLinePrimitive->eType == kGDPLine);

  if (pLinePrimitive->f.line.pPoints) {
    aMemFree(pLinePrimitive->f.line.pPoints);
    pLinePrimitive->f.line.pPoints = (aPT*)NULL;
  }
  
  return err;

} /* sCleanupLine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCleanupText
 */

aErr sCleanupText(
  aGDP* pTextPrimitive)
{
  aErr err = aErrNone;

  aAssert(pTextPrimitive->eType == kGDPText);
  
  if (pTextPrimitive->f.text.pText) {
    aMemFree(pTextPrimitive->f.text.pText);
    pTextPrimitive->f.text.pText = (char*)NULL;
  }

  return err;

} /* sCleanupText */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCleanupCopy
 */

aErr sCleanupCopy(
  aGDP* pCopyPrimitive)
{
  aErr err = aErrNone;

  aAssert(pCopyPrimitive->eType == kGDPCopy);

  if (pCopyPrimitive->f.copy.pCopyGD) {
    aGD_Destroy(pCopyPrimitive->f.copy.pCopyGD->pUI, 
    		(aGDRef)pCopyPrimitive->f.copy.pCopyGD, &err);
    pCopyPrimitive->f.copy.pCopyGD = NULL;
  }

  return err;

} /* sCleanupCopy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sSetFont(
  aUI* pUI, 
  void* vpData,
  aGDP* pTextPrimitive)
{
  aErr err = aErrNone;

  pTextPrimitive->pGD->m_font = pTextPrimitive->f.textdef.def;

  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGDShared_Create
 */

aErr aGDShared_Create(aUI* pUI,
		      void* vpData,
		      const aRECT* pRect,
		      aGD** ppGD)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)NULL;

  if (!ppGD)
    uiErr = aErrParam;

  /* allocate the beast */
  if (uiErr == aErrNone) {
    pGD = (aGD*)aMemAlloc(sizeof(aGD));
    if (pGD) {
      aBZero(pGD, sizeof(aGD));
      pGD->vpData = vpData;
      pGD->r = *pRect;
      pGD->pUI = pUI;
      pGD->textdef = sSetFont;
      pGD->check = aGDCHECK;
      *ppGD = pGD;
    } else
      uiErr = aErrMemory;
  }

  return uiErr;
  
} /* aGDShared_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGDShared_Destroy
 */

aErr aGDShared_Destroy(aGD* pGD)
{
  aErr uiErr = aErrNone;

  if (!pGD)
    uiErr = aErrParam;
  
  /* clean up any outstanding draw list */
  if (uiErr == aErrNone)
    sCleanP(pGD);

  if (uiErr == aErrNone) {
    pGD->check = 0;
    aMemFree(pGD);
  }
  
  return uiErr;
  
} /* aGDShared_Destroy */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGDShared_IssueDrawList
 */

aErr aGDShared_IssueDrawList(aGD* pGD)
{
  aErr err = aErrNone;
  aGDP* pGDP = pGD->pDrawlist;
  
  while ((err == aErrNone) && pGDP) {

    aAssert(pGDP->pCall);
    err = pGDP->pCall(pGD->pUI, pGD->vpData, pGDP);

    /* move to the next primitive */
    pGDP = pGDP->pNext;
  }
  
  return err;

} /* aGDShared_IssueDrawList */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_GetGraphicsMemory
 */

aLIBRETURN 
aGD_GetGraphicsMemory(aUILib uiRef,
		      aGDRef gdRef,
		      char** ppMemory,
		      aErr* pErr)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  aUI* pUI = pGD->pUI;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if (uiErr == aErrNone) {
    if (pGD->getMemory)
      uiErr = pGD->getMemory(pUI, pGD->vpData, ppMemory);
    else
      uiErr = aErrUnimplemented;
  }
  
  if (pErr)
    *pErr = uiErr;
    
  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_GetGraphicsMemory */

     

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_GetSize
 */

aLIBRETURN aGD_GetSize(
  aUILib uiRef,
  aGDRef gdRef,
  aRECT* pRect, 
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }
  if ((uiErr == aErrNone) && !pRect)
    uiErr = aErrParam;

  /* the size is cached in the GD */
  if (uiErr == aErrNone)
    *pRect = pGD->r;
  
  if (pErr)
    *pErr = uiErr;
    
  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_GetSize */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_SetSize
 */

aLIBRETURN aGD_SetSize(
  aUILib uiRef,
  aGDRef gdRef,
  const aRECT* pRect, 
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  aUI* pUI = pGD->pUI;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if (uiErr == aErrNone) {
    pGD->r = *pRect;
    if (pGD->resize)
      uiErr = pGD->resize(pUI, pGD->vpData);
    else
      uiErr = aErrUnimplemented;
  }
  
  if (pErr)
    *pErr = uiErr;
    
  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_SetSize */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_StartDrawing
 */

aLIBRETURN aGD_StartDrawing(aUILib uiRef,
			    aGDRef gdRef,
			    aErr* pErr)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  aUI* pUI = pGD->pUI;
  
  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if (uiErr == aErrNone) {
    if (pGD->startDrawing)
      uiErr = pGD->startDrawing(pUI, pGD->vpData);
    else
      uiErr = aErrUnimplemented;
  }
  
  if (pErr)
    *pErr = uiErr;
    
  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_StartDrawing */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_Erase
 */

aLIBRETURN aGD_Erase(
  aUILib uiRef,
  aGDRef gdRef,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if ((uiErr == aErrNone) && !pGD->erase)
    uiErr = aErrUnimplemented;

  /* dump the draw list */
  if (uiErr == aErrNone)
    sCleanP(pGD);

  /* should invalidate the drawing region */
  if (uiErr == aErrNone) {
    if (pGD->erase)
      uiErr = pGD->erase(pGD->pUI, pGD->vpData);
    else
      uiErr = aErrUnimplemented;
  }

  if (pErr)
    *pErr = uiErr;
    
  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_Erase */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_SetColor
 */

aLIBRETURN aGD_SetColor(
  aUILib uiRef,
  aGDRef gdRef,
  const unsigned long color,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  aGDP* pGDP;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if ((uiErr == aErrNone) && !pGD->setColor)
    uiErr = aErrUnimplemented;

  /* build the list version of the parameters on the list */
  if (uiErr == aErrNone) {
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPSetColor;
    pGDP->f.setColor.color = color;
    sAddP(pGD, pGDP);
    pGDP->pCall = pGD->setColor;
  }

  if (pErr)
    *pErr = uiErr;
    
  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_SetColor */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_Line
 */

aLIBRETURN aGD_Line(
  aUILib uiRef,
  aGDRef gdRef,
  const aPT* pPoints,
  const unsigned int nPoints,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  aGDP* pGDP;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if ((uiErr == aErrNone) && !pGD->line)
    uiErr = aErrUnimplemented;

  /* build the list version of the parameters on the list */
  if (uiErr == aErrNone) {
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPLine;
    pGDP->f.line.nPoints = nPoints;
    pGDP->f.line.pPoints = (aPT*)aMemAlloc(sizeof(aPT) * nPoints);
    if (pGDP->f.line.pPoints) {
      aMemCopy(pGDP->f.line.pPoints, pPoints, sizeof(aPT) * nPoints);
      sAddP(pGD, pGDP);
      pGDP->pCall = pGD->line;
      pGDP->pCleanup = sCleanupLine;
    }
  }

  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_Line */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_Rect
 */

aLIBRETURN aGD_Rect(
  aUILib uiRef,
  aGDRef gdRef,
  const aRECT* pRect,
  const aBool bFilled,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  aGDP* pGDP;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if ((uiErr == aErrNone) && !pGD->rect)
    uiErr = aErrUnimplemented;

  /* build the list version of the parameters on the list */
  if (uiErr == aErrNone) {
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPRect;
    pGDP->f.rect.bounds = *pRect;
    pGDP->f.rect.bFilled = bFilled;
    pGDP->pCall = pGD->rect;
    sAddP(pGD, pGDP);
  }

  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_Rect */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_Elipse
 */

aLIBRETURN aGD_Elipse(
  aUILib uiRef,
  aGDRef gdRef,
  const aRECT* pRect,
  const aBool bFilled,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  aGDP* pGDP;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if ((uiErr == aErrNone) && !pGD->rect)
    uiErr = aErrUnimplemented;

  /* build the list version of the parameters on the list */
  if (uiErr == aErrNone) {
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPElipse;
    pGDP->f.elipse.bounds = *pRect;
    pGDP->f.elipse.bFilled = bFilled;
    pGDP->pCall = pGD->elipse;
    sAddP(pGD, pGDP);
  }

  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_Elipse */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_RoundedRect
 */

aLIBRETURN aGD_RoundedRect(
  aUILib uiRef,
  aGDRef gdRef,
  const aRECT* pRect,
  const aUIPixelType radius,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if ((uiErr == aErrNone) && !(pGD->line || pGD->bezier))
    uiErr = aErrUnimplemented;

  /* do some checking of the absolute size of the rounded rect 
  * checking for degenerate sizes */
  if (uiErr == aErrNone) {
    aUIPixelType t = radius * 2;
    if ((t > pRect->width) || (t > pRect->height))
      uiErr = aErrParam;
  }

  /* now, composite the rounded rectangle from arcs and lines*/
  if (uiErr == aErrNone) {
    aGDP* pGDP;
    aPT p = {0,0};
    aPT p1 = {0,0};

    /* top left to right straight */
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPLine;
    pGDP->f.line.nPoints = 2;
    pGDP->f.line.pPoints = (aPT*)aMemAlloc(sizeof(aPT) * 2);
    if (pGDP->f.line.pPoints) {
      p.x = pRect->x + radius;
      p.y = pRect->y;
      pGDP->f.line.pPoints[0] = p;
      p1.x = pRect->x + pRect->width - radius;
      p1.y = pRect->y;
      pGDP->f.line.pPoints[1] = p1;
      pGDP->pCall = pGD->line;
      pGDP->pCleanup = sCleanupLine;
      sAddP(pGD, pGDP);
    }

    /* top right curve */
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPBezier;
    pGDP->f.bezier.points[0] = p1;
    pGDP->f.bezier.points[1].x = (aUIPixelType)(pRect->x + pRect->width 
      - radius * aBEZIERARCCONTROLRADIUS); 
    pGDP->f.bezier.points[1].y = pRect->y;
    pGDP->f.bezier.points[2].x = pRect->x + pRect->width;
    pGDP->f.bezier.points[2].y = (aUIPixelType)(pRect->y 
      + radius * aBEZIERARCCONTROLRADIUS);
    p1.x = pRect->x + pRect->width;
    p1.y = pRect->y + radius;
    pGDP->f.bezier.points[3] = p1;
    pGDP->pCall = pGD->bezier;
    sAddP(pGD, pGDP);


    /* right top to bottom straight */
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPLine;
    pGDP->f.line.nPoints = 2;
    pGDP->f.line.pPoints = (aPT*)aMemAlloc(sizeof(aPT) * 2);
    if (pGDP->f.line.pPoints) {
      pGDP->f.line.pPoints[0] = p1;
      p1.x = pRect->x + pRect->width;
      p1.y = pRect->y + pRect->height - radius;
      pGDP->f.line.pPoints[1] = p1;
      pGDP->pCall = pGD->line;
      pGDP->pCleanup = sCleanupLine;
      sAddP(pGD, pGDP);
    }

    /* bottom right curve */
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPBezier;
    pGDP->f.bezier.points[0] = p1;
    pGDP->f.bezier.points[1].x = pRect->x + pRect->width; 
    pGDP->f.bezier.points[1].y = (aUIPixelType)(pRect->y + pRect->height 
      - radius * aBEZIERARCCONTROLRADIUS);
    pGDP->f.bezier.points[2].x = (aUIPixelType)(pRect->x + pRect->width 
      - radius * aBEZIERARCCONTROLRADIUS);
    pGDP->f.bezier.points[2].y = (aUIPixelType)(pRect->y + pRect->height);
    p1.x = pRect->x + pRect->width - radius;
    p1.y = pRect->y + pRect->height;
    pGDP->f.bezier.points[3] = p1;
    pGDP->pCall = pGD->bezier;
    sAddP(pGD, pGDP);


    /* bottom right to left straight */
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPLine;
    pGDP->f.line.nPoints = 2;
    pGDP->f.line.pPoints = (aPT*)aMemAlloc(sizeof(aPT) * 2);
    if (pGDP->f.line.pPoints) {
      pGDP->f.line.pPoints[0] = p1;
      p1.x = pRect->x + radius;
      p1.y = pRect->y + pRect->height;
      pGDP->f.line.pPoints[1] = p1;
      pGDP->pCall = pGD->line;
      pGDP->pCleanup = sCleanupLine;
      sAddP(pGD, pGDP);
    }

    /* bottom left curve */
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPBezier;
    pGDP->f.bezier.points[0] = p1;
    pGDP->f.bezier.points[1].x = (aUIPixelType)(pRect->x 
      + radius * aBEZIERARCCONTROLRADIUS); 
    pGDP->f.bezier.points[1].y = (aUIPixelType)(pRect->y + pRect->height);
    pGDP->f.bezier.points[2].x = pRect->x;
    pGDP->f.bezier.points[2].y = (aUIPixelType)(pRect->y + pRect->height
      - radius * aBEZIERARCCONTROLRADIUS);
    p1.x = pRect->x;
    p1.y = pRect->y + pRect->height - radius;
    pGDP->f.bezier.points[3] = p1;
    pGDP->pCall = pGD->bezier;
    sAddP(pGD, pGDP);

    /* left bottom to top straight */
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPLine;
    pGDP->f.line.nPoints = 2;
    pGDP->f.line.pPoints = (aPT*)aMemAlloc(sizeof(aPT) * 2);
    if (pGDP->f.line.pPoints) {
      pGDP->f.line.pPoints[0] = p1;
      p1.x = pRect->x;
      p1.y = pRect->y + radius;
      pGDP->f.line.pPoints[1] = p1;
      pGDP->pCall = pGD->line;
      pGDP->pCleanup = sCleanupLine;
      sAddP(pGD, pGDP);
    }

    /* top left curve */
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPBezier;
    pGDP->f.bezier.points[0] = p1;
    pGDP->f.bezier.points[1].x = pRect->x;
    pGDP->f.bezier.points[1].y = (aUIPixelType)(pRect->y
      + radius * aBEZIERARCCONTROLRADIUS); 
    pGDP->f.bezier.points[2].x = (aUIPixelType)(pRect->x
      + radius * aBEZIERARCCONTROLRADIUS); 
    pGDP->f.bezier.points[2].y = pRect->y;
    pGDP->f.bezier.points[3] = p;
    pGDP->pCall = pGD->bezier;
    sAddP(pGD, pGDP);
  }

  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_RoundedRect */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_Arc
 */

aLIBRETURN aGD_Arc(
  aUILib uiRef,
  aGDRef gdRef,
  const aPT* pCenter,
  const aUIPixelType radius,
  const aUIAngleType startAngle,
  const aUIAngleType arcLength,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if ((uiErr == aErrNone) && !pGD->bezier)
    uiErr = aErrUnimplemented;
  
  if (pGD->bezier) {
    aGDP* pGDP;

    /* build the list version of the parameters on the list */
    if (uiErr == aErrNone) {
      aUIAngleType r1;
      aUIAngleType endAngle;

      sGetP(pGD, &pGDP);
      pGDP->eType = kGDPBezier;

      /* starting point */
      pGDP->f.bezier.points[0].x = 
      	(aUIPixelType)(pCenter->x + radius * cos(startAngle));
      pGDP->f.bezier.points[0].y = 
    	(aUIPixelType)(pCenter->y + radius * sin(startAngle));

      /* conrol point one */
      r1 = (aUIAngleType)(radius * 1.3);
      pGDP->f.bezier.points[1].x = 
    	(aUIPixelType)(pGDP->f.bezier.points[0].x +
    		       r1 * cos(startAngle + aPI/2));
      pGDP->f.bezier.points[1].y = 
    	(aUIPixelType)(pGDP->f.bezier.points[0].y +
    		       r1 * sin(startAngle + aPI/2));

      /* end point */
      endAngle = startAngle + arcLength;
      pGDP->f.bezier.points[3].x = 
    	(aUIPixelType)(pCenter->x + radius * cos(endAngle));
      pGDP->f.bezier.points[3].y = 
    	(aUIPixelType)(pCenter->y + radius * sin(endAngle));

      /* conrol point two */
      pGDP->f.bezier.points[2].x = 
    	(aUIPixelType)(pGDP->f.bezier.points[3].x +
    		       r1 * cos(endAngle - aPI/2));
      pGDP->f.bezier.points[2].y = 
    	(aUIPixelType)(pGDP->f.bezier.points[3].y +
    		       r1 * sin(endAngle - aPI/2));

      pGDP->pCall = pGD->bezier;
      sAddP(pGD, pGDP);
    }
  }

  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_Arc */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_Text
 */

aLIBRETURN aGD_SetFont(
  aUILib uiRef,
  aGDRef gdRef,
  const aFONTDEF* pFontDef,
  aErr* pErr)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;

  if (pErr)
    *pErr = uiErr;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }
  if ((uiErr == aErrNone) && !pFontDef)
    uiErr = aErrParam;

  if ((uiErr == aErrNone) && !pGD->textdef)
    uiErr = aErrUnimplemented;

  if (pGD->textdef) {
    aGDP* pGDP;

    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPSetFont;
    pGDP->pCall = pGD->textdef;

    pGD->m_font = *pFontDef;
    pGDP->f.textdef.def = *pFontDef;

    sAddP(pGD, pGDP);
  }

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_SetFont */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_Text
 */

aLIBRETURN aGD_Text(
  aUILib uiRef,
  aGDRef gdRef,
  const aRECT* pRect,
  const int flags,
  const unsigned int nSize,
  const char* pText,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  aGDP* pGDP;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if ((uiErr == aErrNone) && !pGD->text)
    uiErr = aErrUnimplemented;

  /* build the list version of the parameters on the list */
  if (uiErr == aErrNone) {
    unsigned int len = aStringLen(pText) + 1;
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPText;
    pGDP->f.text.bounds = *pRect;
    pGDP->f.text.flags = flags;
    pGDP->f.text.size = nSize;
    pGDP->f.text.pText = (char*)aMemAlloc(sizeof(char) * len);
    if (pGDP->f.text.pText) {
      aMemCopy(pGDP->f.text.pText, pText, len);
      pGDP->pCall = pGD->text;
      pGDP->pCleanup = sCleanupText;
      sAddP(pGD, pGDP);
    } else
      uiErr = aErrMemory;
  }

  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_Text */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_Paragraph
 */

aLIBRETURN aGD_Paragraph(
  aUILib uiRef,
  aGDRef gdRef,
  const aRECT* pRect,
  const int flags,
  const unsigned int nSize,
  char** ppText,
  unsigned int* pnLines,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  aRECT r;
  int len = 0;
  unsigned int width;
  int nLines = 0;
  int nMaxLines = pRect->height / nSize;
  char save = 0;
  char* pText = NULL;
  char* pLineStart;
  char* pLineFits;
  aBool bFirstChar;
  char* p;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if ((uiErr == aErrNone) && (!pGD->text || !pGD->textWidth))
    uiErr = aErrUnimplemented;

  /* create a copy of the text so we can modify it */
  if (uiErr == aErrNone) {
    aMemSize length = sizeof(char) * (aStringLen(*ppText) + 1);
    pText = (char*)aMemAlloc(length);
    if (pText)
      aMemCopy(pText, *ppText, length);
    else
      uiErr = aErrMemory;
  }

  if (uiErr == aErrNone) {
    p = pText;
    r = *pRect;
    r.height = nSize;
    while (*p && (nLines < nMaxLines)) {
      bFirstChar = aFalse;
      width = 0;
      pLineStart = p;
      pLineFits = p;

      /* gobble words up until the line no longer fits */
      while (*p && (width < (unsigned int)pRect->width)) {

        switch (*p) {

        /* we got a word break */
        case '\n':
        case ' ':
          if (bFirstChar || (*p == ' ')) {
	    /* if this isn't the head of the line, calculate
	     * the line width by temporarily terminating
	     * the line.  If it fits, advance the end-of-line
	     * pointer to just beyond the word.  If not,
	     * reduce the line length by the word length 
	     * which didn't fit. */
            save = *p;
            *p = 0;
            aGD_TextWidth(uiRef, gdRef, pLineStart, nSize, &width, &uiErr);
            if (width < (unsigned int)pRect->width)
              pLineFits = p;
            else if (pLineFits)
              len -= aStringLen(pLineFits);
            *p = save;
          }

	  /* end of line forces a break */
	  if (save == '\n')
	    width = pRect->width;
          break;

        default:
	  /* any other (non-white space) character is 
	   * considered a first character */
          bFirstChar = aTrue;
          break;

        } /* switch */
        p++;
        len++;
      } /* while */

      /* handle the last word of the line */
      if (!(*p) && bFirstChar) {
	aGD_TextWidth(uiRef, gdRef, pLineStart, nSize, 
		      &width, &uiErr);
	if (width < (unsigned int)pRect->width) {
	  pLineFits = p;
	  len = -1;
	  *ppText = p;
	} else if (pLineFits) {
	  len -= aStringLen(pLineFits);
	  if (*pLineFits) {
	    *pLineFits = 0;
	    p = pLineFits + 1;
	  }
	}

      /* or, prepare the line */
      } else {
	if (width)
	  *pLineFits = 0; /* terminate the line */

	/* done with this line, advance to the next line */
	if (bFirstChar)
	  p = pLineFits + 1;
      }

      /* now, pLineStart points to the line to draw and the 
       * text is terminated. */

      /* draw the line if it is visible */
      if (bFirstChar && !(flags & aUITEXTINVISIBLE))
        aGD_Text(uiRef, gdRef, &r, flags, nSize, 
        	 pLineStart, &uiErr);

      /* advance to the next line */
      r.y += nSize;
      nLines++;

    } /* while */
  } /* if uiErr == aErrNone */
  
  /* advance the text pointer to the */
  if ((uiErr == aErrNone) && (len != -1))
    (*ppText) += len;

  /* set the number of lines if requested */
  if ((uiErr == aErrNone) && pnLines)
    *pnLines = nLines;

  if (pText)
    aMemFree(pText);

  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_Paragraph */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_Copy
 *
 * This should handle the clipping of the source and the destination
 * regions.  Currently it does not handle this.
 */

aLIBRETURN aGD_Copy(
  aUILib uiRef,
  aGDRef gdRef,
  const aRECT* pDstRect,
  aGDRef srcGDRef,
  const aRECT* pSrcRect,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  aGD* pSrcGD = (aGD*)srcGDRef;
  aGDP* pGDP;
  aGD* pCopyGD = NULL;

  aVALIDUI(uiRef);
  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if ((uiErr == aErrNone) && (!pGD->copy || !pSrcGD->getPixels))
    uiErr = aErrUnimplemented;

  /* build the storage for the copied pixels */    
  if (uiErr == aErrNone) {
    aGDRef r;
    aGDInternal_CreateOffscreen(pGD->pUI, 
				pSrcRect->width,
			        pSrcRect->height,
				&r,
				&uiErr);
    pCopyGD = (aGD*)r;
  }

    /* copy the pixels into the new storage GD */
  if (uiErr == aErrNone) {
      aGDOS* pGDOS = (aGDOS*)pCopyGD->vpData;
      uiErr = pSrcGD->getPixels(pSrcGD->pUI,
		                pSrcGD->vpData,
		                pSrcRect,
		                pGDOS->pPixels);
  }

  if (uiErr == aErrNone) {
    sGetP(pGD, &pGDP);
    pGDP->eType = kGDPCopy;
    pGDP->f.copy.dest = *pDstRect;
    pGDP->f.copy.pCopyGD = pCopyGD;
    pGDP->pCall = pGD->copy;
    pGDP->pCleanup = sCleanupCopy;
    sAddP(pGD, pGDP);
  }

  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_Copy */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_TextWidth
 */

aLIBRETURN aGD_TextWidth(
  aUILib uiRef,
  aGDRef gdRef,
  const char* pText,
  const unsigned int nSize,
  unsigned int* pnWidth,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;

  aVALIDUI(uiRef);
  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }
  if ((uiErr == aErrNone) && (!pText || !nSize))
    uiErr = aErrParam;

  if ((uiErr == aErrNone) && !pGD->textWidth)
    uiErr = aErrUnimplemented;

  if (uiErr == aErrNone) {
    uiErr = pGD->textWidth(pGD->pUI, 
    			   pGD->vpData,
		           pText,
		           nSize,
		           pnWidth);
  }

  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_TextWidth */ 



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_EndDrawing
 */

aLIBRETURN aGD_EndDrawing(aUILib uiRef,
			  aGDRef gdRef,
			  aErr* pErr)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  if (uiErr == aErrNone) {
    if (pGD->endDrawing)
      uiErr = pGD->endDrawing(pGD->pUI, pGD->vpData);
    else
      uiErr = aErrUnimplemented;
  }

  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_EndDrawing */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_Destroy
 */

aLIBRETURN aGD_Destroy(aUILib uiRef,
                       aGDRef gdRef,
		       aErr* pErr) 
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  
  aVALIDUI(uiRef);

  if (uiErr == aErrNone) {
    aVALIDGD(pGD);
  }

  /* clean up any remaining draw list info */
  if (uiErr == aErrNone)
    sCleanP(pGD);

  if (uiErr == aErrNone) {
    if (pGD->destroy)
      uiErr = pGD->destroy(pGD->pUI, pGD->vpData);
    else
      uiErr = aErrUnimplemented;
    pGD->vpData = NULL;
  }
  
  if (pGD) {
    pGD->check = 0;
    aMemFree(pGD);
  }

  if (pErr)
    *pErr = uiErr;
    
  return (aLIBRETURN)(uiErr != aErrNone);
  
} /* aGD_Destroy */
