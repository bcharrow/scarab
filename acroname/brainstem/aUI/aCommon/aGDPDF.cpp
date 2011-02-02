/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aGDPDF.cpp                                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* descriptUIn: Implementation of a platform-independent graphics  */
/*		PDF drawing object.				   */
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

#ifndef aPALM

#include "aUIInternal.h"
#include "acpGDPDF.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static aErr sGDPDF_Resize(
  aUI* pUI, 
  void* vpData);
static aErr sGDPDF_StartDrawing(
  aUI* pUI, 
  void* vpData);
static aErr sGDPDF_Erase(
  aUI* pUI, 
  void* vpData);
static aErr sGDPDF_TextWidth(
  aUI* pUI, 
  void* vpData,
  const char* pText,
  const unsigned int nSize,
  unsigned int* pnWidth);
static aErr sGDPDF_SetColor(
  aUI* pUI, 
  void* vpData,
  aGDP* pSetColorPrimitive);
static aErr sGDPDF_Line(
  aUI* pUI, 
  void* vpData,
  aGDP* pSetLinePrimitive);
static aErr sGDPDF_Rect(
  aUI* pUI, 
  void* vpData,
  aGDP* pRectPrimitive);
static aErr sGDPDF_Bezier(
  aUI* pUI, 
  void* vpData,
  aGDP* pBezierPrimitive);
static aErr sGDPDF_Text(
  aUI* pUI, 
  void* vpData,
  aGDP* pTextPrimitive);
static aErr sGDPDF_Copy(
  aUI* pUI, 
  void* vpData,
  aGDP* pCopyPrimitive);
static aErr sGDPDF_EndDrawing(
  aUI* pUI, 
  void* vpData);
static aErr sGDPDF_Destroy(
  aUI* pUI, 
  void* vpData);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aLIBRETURN aGD_CreatePDF(
  aUILib uiRef,
  const char* pFileName,
  const aFileArea eFileArea,
  aGDRef* pGDRef,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aUI* pUI = (aUI*)uiRef;
  aGD* pGD = (aGD*)NULL;
  acpGDPDF* pGDPDF = (acpGDPDF*)NULL;

  aVALIDUI(pUI);

  if ((uiErr == aErrNone) && !pGDRef)
    uiErr = aErrParam;
  if ((uiErr == aErrNone) && !pFileName)
    uiErr = aErrRange;

  /* initialize */
  if (pGDRef)
    *pGDRef = 0;

  /* allocate the PDF data */
  if (uiErr == aErrNone)
    pGDPDF = new acpGDPDF(pUI->ioRef, pFileName, eFileArea);

  /* allocate the GD */
  if (uiErr == aErrNone) {
    aRECT r;
    r.x = 0;
    r.y = 0;
    r.width = (aUIPixelType)pGDPDF->width();
    r.height = (aUIPixelType)pGDPDF->height();  
    uiErr = aGDShared_Create(pUI, pGDPDF, &r, &pGD);
    pGDPDF->setGD(pGD);
  }

  /* set up the offscreen drawing routines for this GD */
  if (uiErr == aErrNone) {
    pGD->resize       = sGDPDF_Resize;
    pGD->startDrawing = sGDPDF_StartDrawing;
    pGD->erase        = sGDPDF_Erase;
    pGD->textWidth    = sGDPDF_TextWidth;
    pGD->setColor     = sGDPDF_SetColor;
    pGD->line         = sGDPDF_Line;
    pGD->rect         = sGDPDF_Rect;
    pGD->bezier       = sGDPDF_Bezier;
    pGD->text         = sGDPDF_Text;
    pGD->copy	      = sGDPDF_Copy;
    pGD->endDrawing   = sGDPDF_EndDrawing;
    pGD->destroy      = sGDPDF_Destroy;
  }

  /* clean up if we failed */
  if (uiErr == aErrNone) {
    *pGDRef = pGD;
  } else {
    sGDPDF_Destroy(pUI, pGD);
    if (pGD)
      aGDShared_Destroy(pGD);
  }

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_Resize(
  aUI* pUI, 
  void* vpData
)
{
  aErr uiErr = aErrNone;
  acpGDPDF* pGDPDF = (acpGDPDF*)vpData;

  if (uiErr == aErrNone) {
    aGD* pGD = pGDPDF->getGD();
    pGDPDF->setSize(pGD->r.width, pGD->r.height);
  }

  return uiErr;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_StartDrawing(
  aUI* pUI, 
  void* vpData
)
{
  aErr err = aErrNone;
  
  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_Erase(
  aUI* pUI, 
  void* vpData
)
{
  aErr uiErr = aErrNone;
  acpGDPDF* pGDPDF = (acpGDPDF*)vpData;

  if (uiErr == aErrNone)
    pGDPDF->nextPage();

  return uiErr;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_TextWidth(
  aUI* pUI, 
  void* vpData,
  const char* pText,
  const unsigned int nSize,
  unsigned int* pnWidth
)
{
  aErr uiErr = aErrNone;
  acpGDPDF* pGDPDF = (acpGDPDF*)vpData;

  if (uiErr == aErrNone)
    *pnWidth = pGDPDF->textWidth(pText, nSize);

  return uiErr;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_SetColor(
  aUI* pUI, 
  void* vpData,
  aGDP* pSetColorPrimitive
)
{
  aErr err = aErrNone;
  acpGDPDF* pGDPDF = (acpGDPDF*)vpData;

  pGDPDF->setColor(pSetColorPrimitive);

  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_Line(
  aUI* pUI, 
  void* vpData,
  aGDP* pLinePrimitive
)
{
  aErr err = aErrNone;
  acpGDPDF* pGDPDF = (acpGDPDF*)vpData;

  pGDPDF->line(pLinePrimitive);

  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_Rect(
  aUI* pUI, 
  void* vpData,
  aGDP* pRectPrimitive
)
{
  aErr err = aErrNone;
  acpGDPDF* pGDPDF = (acpGDPDF*)vpData;

  pGDPDF->rect(pRectPrimitive);

  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_Bezier(
  aUI* pUI, 
  void* vpData,
  aGDP* pBezierPrimitive
)
{
  aErr err = aErrNone;
  acpGDPDF* pGDPDF = (acpGDPDF*)vpData;

  pGDPDF->bezier(pBezierPrimitive);
  
  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_Text(
  aUI* pUI, 
  void* vpData,
  aGDP* pTextPrimitive
)
{
  aErr err = aErrNone;
  acpGDPDF* pGDPDF = (acpGDPDF*)vpData;

  pGDPDF->text(pTextPrimitive);

  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_Copy(
  aUI* pUI, 
  void* vpData,
  aGDP* pCopyPrimitive
)
{
  aErr err = aErrNone;
  acpGDPDF* pGDPDF = (acpGDPDF*)vpData;

  pGDPDF->image(pCopyPrimitive);

  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_EndDrawing(
  aUI* pUI, 
  void* vpData
)
{
  acpGDPDF* pGDPDF = (acpGDPDF*)vpData;

  return aGDShared_IssueDrawList(pGDPDF->getGD());
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDPDF_Destroy(
  aUI* pUI, 
  void* vpData
)
{
  aErr err = aErrNone;
  acpGDPDF* pGDPDF = (acpGDPDF*)vpData;

  delete pGDPDF;

  return err;
}

#endif /* aPALM */
