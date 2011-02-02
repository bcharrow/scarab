/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aGD.c                                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: implementation of UI routines for unix.            */
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

#ifdef aUNIX

#include "unix_aUI.h"
#include "aGD.h"


typedef struct aUnixGD {
  aGD*                  pGD;
#ifndef aNOX
  Window                m_drawable;
  Display*              m_pDisplay;
  GC                    m_gc;
  XFontStruct*          m_font;
#endif /* aNOX */
  unsigned int          m_fontsize;
  int                   m_decent;
  unsigned long         m_curColor;
  struct aUnixGD*       m_pNextGD;
} aUnixGD;


static aErr unix_Resize(
  aUI* pUI,
  void* vpData);
static aErr unix_StartDrawing(
  aUI* pUI,
  void* vpData);

#ifndef aNOX
static aErr unix_Erase(
  aUI* pUI,
  void* vpData);
static aErr unix_TextWidth(
  aUI* pUI, 
  void* vpData,
  const char* pText,
  const unsigned int nSize,
  unsigned int* pnWidth);
static aErr unix_SetColor(
  aUI* pUI,
  void* vpData,
  aGDP* pSetColorPrimitive);
static aErr unix_Line(
  aUI* pUI,
  void* vpData,
  aGDP* pLinePrimitive);
static aErr unix_Rect(
  aUI* pUI,
  void* vpData,
  aGDP* pRectPrimitive);
static aErr unix_Text(
  aUI* pUI,
  void* vpData,
  aGDP* pTextPrimitive);
static aErr unix_Copy(
  aUI* pUI,
  void* vpData,
  aGDP* pCopyPrimitive);
static aErr unix_Bezier(
  aUI* pUI,
  void* vpData,
  aGDP* pBezierPrimitive);
static aErr sSetFont(
  aUnixGD* pUnixGD,
  const unsigned int size);
#endif /* !aNOX */

static aErr unix_EndDrawing(
  aUI* pUI,
  void* vpData);
static aErr unix_Destroy(
  aUI* pUI,
  void* vpData);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_Create
 */

aLIBRETURN aGD_Create(
  aUILib uiRef,
  const aRECT* pRect,
  void* createData,
  aGDRef* pGDRef,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aUnixUI* pUnixUI = (aUnixUI*)uiRef;
  aGD* pGD = (aGD*)NULL;
  aUnixGD* pUnixGD = NULL;

#ifndef aNOX
  Display* pDisplay = NULL;
  Window parent = 0;
#endif /* aNOX */

  aVALIDUI(uiRef);

#ifndef aNOX
  if (uiErr == aErrNone) {
    if ((pRect->width <= 0) || (pRect->height <= 0)) {
#ifdef aDEBUG
      printf("Error creating invalid GD with size %dx%d\n",
	     pRect->width, pRect->height);
#endif /* aDEBUG */
      uiErr = aErrParam;
    } else {
      aUnixGDCreateData* pCD = (aUnixGDCreateData*)createData;
      pDisplay = pCD->pDisplay;
      parent = pCD->parent;
    }
  }

  /* initialize X windows only once */
  if ((uiErr == aErrNone) && !pUnixUI->bXInited) {
    pUnixUI->bXInited = 1;/* show that we are initialized */
  }
#endif /* aNOX */

  /* build the unix-specific data structure for the GD */
  if (uiErr == aErrNone) {
    pUnixGD = (aUnixGD*)aMemAlloc(sizeof(aUnixGD));
    if (pUnixGD) {
      aBZero(pUnixGD, sizeof(aUnixGD));
#ifndef aNOX
      pUnixGD->m_pDisplay = pDisplay;
#endif /* aNOX */
    } else 
      uiErr = aErrMemory;
  }

  if (uiErr == aErrNone)
    uiErr = aGDShared_Create((aUI*)uiRef, (void*)pUnixGD, pRect, &pGD);

#ifndef aNOX
  if (uiErr == aErrNone) {
    pUnixGD->m_drawable = XCreateSimpleWindow(pDisplay,
					      parent,
					      pRect->x,
					      pRect->y,
					      pRect->width,
					      pRect->height,
					      0, /* border width */
					      0, /* border */
					      0xFFFFFF); /* background */

    /* try building the GC once and using it for the duration */
    {
      XGCValues GCValues;
      GCValues.foreground = 0x000000;
      GCValues.background = 0xFFFFFF;
      
      pUnixGD->m_gc = XCreateGC(pUnixGD->m_pDisplay,
				pUnixGD->m_drawable,
				GCForeground | GCBackground,
				&GCValues);
    }

    /* show the window */
    XSelectInput(pDisplay, pUnixGD->m_drawable, 
		 ExposureMask 
		 | ResizeRequest
		 | VisibilityChangeMask
		 | FocusChangeMask);
    XMapWindow(pDisplay, pUnixGD->m_drawable);
  }
#endif /* aNOX */

  /* set up the callbacks */
  if (uiErr == aErrNone) {
    pGD->resize       = unix_Resize;
    pGD->startDrawing = unix_StartDrawing;

#ifndef aNOX
    pGD->erase        = unix_Erase;
    pGD->textWidth    = unix_TextWidth;
    pGD->setColor     = unix_SetColor;
    pGD->line         = unix_Line;
    pGD->rect         = unix_Rect;
    pGD->text         = unix_Text;
    pGD->copy         = unix_Copy;
    pGD->bezier       = unix_Bezier;
#endif /* aNOX */

    pGD->endDrawing   = unix_EndDrawing;
    pGD->destroy      = unix_Destroy;
    *pGDRef = pGD;

    /* link it into our list of drawables */
    pUnixGD->m_pNextGD = pUnixUI->pGDList;
    pUnixUI->pGDList = pUnixGD;
  }

  if (uiErr == aErrNone)
    pUnixGD->pGD = pGD;

  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_Resize
 */

aErr unix_Resize(
  aUI* pUI,
  void* vpData)
{
  aErr uiErr = aErrNone;

#ifndef aNOX
  aUnixGD* pUnixGD = (aUnixGD*)vpData;

  if ((uiErr == aErrNone) && pUnixGD->m_drawable) {
    aGD* pGD = pUnixGD->pGD;
    XMoveResizeWindow(pUnixGD->m_pDisplay,
		      pUnixGD->m_drawable,
		      pGD->r.x,
		      pGD->r.y,
		      pGD->r.width,
		      pGD->r.height);
  }
#endif /* aNOX */
 
  return uiErr;

} /* unix_Resize */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_StartDrawing
 */

aErr unix_StartDrawing(
  aUI* pUI,
  void* vpData)
{
  aErr uiErr = aErrNone;

  return uiErr;

} /* unix_StartDrawing */


#ifndef aNOX
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_Erase
 */

aErr unix_Erase(
  aUI* pUI,
  void* vpData)
{
  aErr uiErr = aErrNone;

  aUnixGD* pUnixGD = (aUnixGD*)vpData;

  /* blast the background of the window */
  XSetForeground(pUnixGD->m_pDisplay, pUnixGD->m_gc, 
		 0xFFFFFF);
  XFillRectangle(pUnixGD->m_pDisplay,
		 pUnixGD->m_drawable,
		 pUnixGD->m_gc,
		 0,
		 0,
		 pUnixGD->pGD->r.width,
		 pUnixGD->pGD->r.height);
  XSetForeground(pUnixGD->m_pDisplay, pUnixGD->m_gc, 
		 pUnixGD->m_curColor);

  return uiErr;

} /* unix_Erase */
#endif


#ifndef aNOX
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_TextWidth
 */

aErr unix_TextWidth(
  aUI* pUI, 
  void* vpData,
  const char* pText,
  const unsigned int nSize,
  unsigned int* pnWidth)
{
  aErr uiErr = aErrNone;
  aUnixGD* pUnixGD = (aUnixGD*)vpData;

  if (uiErr == aErrNone)
    uiErr = sSetFont(pUnixGD, nSize);

  if (uiErr == aErrNone) {
    int ascent;
    int direction;
    int descent;
    XCharStruct cs;
    XTextExtents(pUnixGD->m_font, 
		 pText, 
		 aStringLen(pText), 
		 &direction,
		 &ascent,
		 &descent,
		 &cs);
    *pnWidth = cs.width;
  }

  return uiErr;

} /* unix_TextWidth */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_SetColor
 */

aErr unix_SetColor(
  aUI* pUI,
  void* vpData,
  aGDP* pSetColorPrimitive)
{
  aErr uiErr = aErrNone;
  aUnixGD* pUnixGD = (aUnixGD*)vpData;

  aAssert(pUnixGD->m_gc);
  XSetForeground(pUnixGD->m_pDisplay, pUnixGD->m_gc, 
		 pSetColorPrimitive->f.setColor.color);
  pUnixGD->m_curColor = pSetColorPrimitive->f.setColor.color;

  return uiErr;

} /* unix_SetColor */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_Line
 */

aErr unix_Line(
  aUI* pUI,
  void* vpData,
  aGDP* pLinePrimitive)
{
  aErr uiErr = aErrNone;
  aUnixUI* pUnixUI = (aUnixUI*)pUI;
  aUnixGD* pUnixGD = (aUnixGD*)vpData;
  unsigned int nPoints = pLinePrimitive->f.line.nPoints;

  if ((nPoints > 1) && (uiErr == aErrNone)) {
    if (nPoints > pUnixUI->nPointBufferSize) {
      if (pUnixUI->pPointBuffer)
        aMemFree(pUnixUI->pPointBuffer);
      pUnixUI->pPointBuffer = (XPoint*)aMemAlloc(nPoints * sizeof(XPoint));
      if (pUnixUI->pPointBuffer == NULL)
	uiErr = aErrMemory;
      else
	pUnixUI->nPointBufferSize = nPoints;
    }

    /* convert to Unix point structures */
    if (uiErr == aErrNone) {
      const aPT* s = pLinePrimitive->f.line.pPoints;
      XPoint* p = pUnixUI->pPointBuffer;
      unsigned int i;

      for (i = nPoints; i--; s++, p++) {
	p->x = s->x;
	p->y = s->y;
      }

      XDrawLines(pUnixGD->m_pDisplay,
		 pUnixGD->m_drawable,
		 pUnixGD->m_gc,
		 pUnixUI->pPointBuffer,
		 nPoints,
		 CoordModeOrigin);
    }
  }

  return uiErr;

} /* unix_Line */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_Rect
 */

aErr unix_Rect(
  aUI* pUI,
  void* vpData,
  aGDP* pRectPrimitive)
{
  aErr uiErr = aErrNone;
  aUnixGD* pUnixGD = (aUnixGD*)vpData;
  aRECT* pRect;

  if ((uiErr == aErrNone) && !pRectPrimitive)
    uiErr = aErrParam;

  aAssert(pRectPrimitive->eType == kGDPRect);

  pRect = &pRectPrimitive->f.rect.bounds;

  if (uiErr == aErrNone) {
    if (pRectPrimitive->f.rect.bFilled) {
      XFillRectangle(pUnixGD->m_pDisplay,
		     pUnixGD->m_drawable,
		     pUnixGD->m_gc,
		     pRect->x,
		     pRect->y,
		     pRect->width,
		     pRect->height);
    } else {
      XDrawRectangle(pUnixGD->m_pDisplay,
		     pUnixGD->m_drawable,
		     pUnixGD->m_gc,
		     pRect->x,
		     pRect->y,
		     pRect->width,
		     pRect->height);
    }
  }

  return uiErr;

} /* unix_Rect */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_Text
 */

aErr unix_Text(
  aUI* pUI,
  void* vpData,
  aGDP* pTextPrimitive)
{
  aErr uiErr = aErrNone;
  aUnixGD* pUnixGD = (aUnixGD*)vpData;
  aRECT* pRect;
  const char* pText;

  aAssert(pTextPrimitive->eType == kGDPText);

  if (uiErr == aErrNone)
    uiErr = sSetFont(pUnixGD, pTextPrimitive->f.text.size);

  if (uiErr == aErrNone) {
    pRect = &pTextPrimitive->f.text.bounds;
    pText = pTextPrimitive->f.text.pText;
  }

  if (uiErr == aErrNone) {
    int ascent;
    int direction;
    int decent;
    XCharStruct cs;
    int len = strlen(pText);
    int left = pRect->x;
    
    XTextExtents(pUnixGD->m_font, 
		 pText, 
		 len, 
		 &direction,
		 &ascent,
		 &decent,
		 &cs);
    
    switch (pTextPrimitive->f.text.flags) {
    case aUIALIGNLEFT:
      break;
    case aUIALIGNCENTER:
      left += (pRect->width - cs.width) / 2;
      break;
    case aUIALIGNRIGHT:
      left += pRect->width - cs.width;
      break;
    } /* switch */

    XDrawString(pUnixGD->m_pDisplay,
		pUnixGD->m_drawable,
		pUnixGD->m_gc,
		left,
		pRect->y + ((pRect->height + pTextPrimitive->f.text.size) 
			    / 2) - decent,
		pText,
		strlen(pText));
  }


  return uiErr;

} /* unix_Text */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_Copy
 */

aErr unix_Copy(
  aUI* pUI,
  void* vpData,
  aGDP* pCopyPrimitive)
{
  aErr uiErr = aErrNone;

#if 0
  aUnixGD* pUnixGD = (aUnixGD*)vpData;
  HDC sourceDC = NULL;
  BITMAPINFO bmi;
  HBITMAP hBuffer = NULL;
  char* pSrcPixels;
  char* pDstPixels;
  gdGetMemory getMemory;
  aRECT srcRect = pCopyPrimitive->f.copy.pCopyGD->r;

  // set up a DC for the source pixels in memory
  bmi.bmiHeader.biSize = sizeof(BITMAPINFO);
  bmi.bmiHeader.biWidth = srcRect.width;
  bmi.bmiHeader.biHeight = srcRect.height;
  bmi.bmiHeader.biPlanes = 1; // number of planes (must be 1)
  bmi.bmiHeader.biBitCount = 24; // bits per pixel
  bmi.bmiHeader.biCompression = BI_RGB; // RGB, uncompressed
  bmi.bmiHeader.biSizeImage = 0; // zero for RGB
  bmi.bmiHeader.biXPelsPerMeter = 0;
  bmi.bmiHeader.biXPelsPerMeter = 0;
  bmi.bmiHeader.biClrUsed = 0;
  bmi.bmiHeader.biClrImportant = 0;
  
  sourceDC = CreateCompatibleDC(pWin32GD->hDC);
  if (!sourceDC)
    uiErr = aErrIO;

  /* find the proc ptr for getting the source pixels */
  if (uiErr == aErrNone) {
    getMemory = pCopyPrimitive->f.copy.pCopyGD->getMemory;
    aAssert(getMemory);
    if (!getMemory)
      uiErr = aErrParam;
  }

  /* get the source pixels */
  if (uiErr == aErrNone)
    uiErr = getMemory(pUI, pCopyPrimitive->f.copy.pCopyGD->vpData, &pSrcPixels);
      

  // build the windows copy buffer
  if (uiErr == aErrNone)
    // build the offscreen bitmap for storing the pixel data
    hBuffer = CreateDIBSection(sourceDC,
  			       &bmi,
  			       DIB_RGB_COLORS,
  			       (void**)&pDstPixels,
  			       NULL,
  			       0L);

  // dump the image into the copy buffer from the source
  // this could be more efficient (not double copy) but this is a start
  if (uiErr == aErrNone) {
    // Here, we copy the src data to the hBuffer's memory.  In doing this
    // we flip rgb to bgr for Windoz.
    char* s = pSrcPixels;
    char* d = pDstPixels;
    int i;
    for (i = 0; i < srcRect.width * srcRect.height; i++, s+=3, d+=3) {
      d[0] = s[2];
      d[1] = s[1];
      d[2] = s[0];
    }
    SelectObject(sourceDC, hBuffer);
    if (!StretchBlt(pWin32GD->hDC, 
  	            pCopyPrimitive->f.copy.dest.x,
  	            pCopyPrimitive->f.copy.dest.y,
  	            pCopyPrimitive->f.copy.dest.width,
  	            pCopyPrimitive->f.copy.dest.height,
  	            sourceDC,
  	            0,
  	            srcRect.height - 1,
  	            srcRect.width,
  	            -srcRect.height,
  	            SRCCOPY)) {
      uiErr = aErrUnknown;
    }
  }

  if (hBuffer)
    DeleteObject(hBuffer);
 
  if (sourceDC)
    DeleteDC(sourceDC);
#endif 
  return uiErr;
} /* unix_Copy */



 /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_Bezier
 */

aErr unix_Bezier(
  aUI* pUI,
  void* vpData,
  aGDP* pBezierPrimitive)
{
  aErr uiErr = aErrNone;
  //  aUnixGD* pUnixGD = (aUnixGD*)vpData;

  aAssert(pBezierPrimitive->eType == kGDPBezier);

#if 0
  aREADYTODRAW(pWin32GD)

  if (uiErr == aErrNone) {
    aPT* p = pBezierPrimitive->f.bezier.points;
    POINT wp[4];
    wp[0].x = p[0].x;
    wp[0].y = p[0].y;
    wp[1].x = p[1].x;
    wp[1].y = p[1].y;
    wp[2].x = p[2].x;
    wp[2].y = p[2].y;
    wp[3].x = p[3].x;
    wp[3].y = p[3].y;
    PolyBezier(pWin32GD->hDC, wp, 4);
  }
#endif

  return uiErr;

} /* unix_Bezier */
#endif /* aNOX */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_EndDrawing
 */

aErr unix_EndDrawing(
  aUI* pUI,
  void* vpData)
{
  aErr uiErr = aErrNone;

#ifndef aNOX
  aUnixGD* pUnixGD = (aUnixGD*)vpData;

  if (uiErr == aErrNone) {
    aAssert(pUnixGD->m_gc);
    uiErr = aGDShared_IssueDrawList(pUnixGD->pGD);
    // here, false means don't flush event queue
    XSync(pUnixGD->m_pDisplay, aFalse);
  }
#endif /* aNOX */

  return uiErr;

} /* unix_EndDrawing */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_Destroy
 */

aErr unix_Destroy(
  aUI* pUI,
  void* vpData)
{
  aErr uiErr = aErrNone;
  aUnixGD* pUnixGD = (aUnixGD*)vpData;
  aUnixUI* pUnixUI = (aUnixUI*)pUI;

  /* remove ourselves from the list of GD's in the aUI record */
  aUnixGD* pTemp = pUnixUI->pGDList;
  aUnixGD* pPrev = NULL;
  while (pTemp && !(pTemp == pUnixGD)) {
    pPrev = pTemp;
    pTemp = pTemp->m_pNextGD;
  }
  aAssert(pTemp); /* must find ourselves */
  if (pPrev)
    pPrev->m_pNextGD = pTemp->m_pNextGD;
  else
    pUnixUI->pGDList = pTemp->m_pNextGD;

#ifndef aNOX
  /* destroy the GC once */
  if ((uiErr == aErrNone) && pUnixGD->m_gc) {
    XFreeGC(pUnixGD->m_pDisplay, pUnixGD->m_gc);
    pUnixGD->m_gc = NULL;
  }

  if ((uiErr == aErrNone) && pUnixGD->m_font)
    XFreeFont(pUnixGD->m_pDisplay, pUnixGD->m_font);

  if (pUnixGD && pUnixGD->m_drawable) {
    aAssert(pUnixGD->m_pDisplay);
    XDestroyWindow(pUnixGD->m_pDisplay, pUnixGD->m_drawable);
    pUnixGD->m_drawable = 0;
  }
#endif /* aNOX */

  if (pUnixGD && (uiErr == aErrNone))
    aMemFree(pUnixGD);

  return uiErr;

} /* unix_Destroy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_HandleEvent
 */

aLIBRETURN aGD_HandleEvent(
  aUILib uiRef,
  void* pXEvent,
  aBool* pbHandled,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aBool bHandled = aFalse;

#ifndef aNOX
  aUnixUI* pUnixUI = (aUnixUI*)uiRef;
  XEvent* pEvent = (XEvent*)pXEvent;
#endif /* aNOX */

  aVALIDUI(uiRef);

#ifndef aNOX
  if (uiErr == aErrNone) {
    switch (pEvent->type) {

    case GraphicsExpose:
    case Expose:
      bHandled = aTrue;

      /* if we have several waiting, don't redraw just do the rendering
       * on the last one */
      if (pEvent->xexpose.count == 0) {
	aUnixGD* pGD = pUnixUI->pGDList;
	while (pGD && (pGD->m_drawable != pEvent->xexpose.window))
	  pGD = pGD->m_pNextGD;

	if (pGD) {
	  /* may want to erase here */
	  uiErr = aGDShared_IssueDrawList(pGD->pGD);
	  aAssert(pGD->m_gc);
	  XSync(pGD->m_pDisplay, pGD->m_drawable);
	}
      }
      break;
    } /* switch on event type */
  }
#endif /* aNOX */

  if (pbHandled)
    *pbHandled = bHandled;

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_HandleEvent */


#ifndef aNOX
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSetFont
 */

aErr sSetFont(
  aUnixGD* pUnixGD,
  const unsigned int size)
{
  aErr uiErr = aErrNone;

  if ((uiErr == aErrNone) && (pUnixGD->m_fontsize != size)) {
    char fontname[100];

    if (pUnixGD->m_font) {
      XFreeFont(pUnixGD->m_pDisplay, pUnixGD->m_font);
      pUnixGD->m_font = NULL;
    }

    sprintf(fontname, "-*-helvetica-*-*-*--%d-*-*-*-*-*-*", size);
    pUnixGD->m_font = XLoadQueryFont(pUnixGD->m_pDisplay, fontname);
    if (!pUnixGD->m_font) {
      /*      printf("unable to load font named %s\n", fontname); */
      pUnixGD->m_font = XLoadQueryFont(pUnixGD->m_pDisplay, "Fixed");
    }
    pUnixGD->m_fontsize = size;
  }

  return uiErr;
}
#endif /* aNOX */

#endif /* aUNIX */
