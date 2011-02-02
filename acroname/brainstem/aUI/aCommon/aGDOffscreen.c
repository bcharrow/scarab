/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aGDOffscreen.c                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent graphics  */
/*		offscreen drawing object.			   */
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
#include "aFont.h"


extern aFont font_8;
extern aFont font_9;
extern aFont font_10;
extern aFont font_11;
extern aFont font_12;
extern aFont font_14;
extern aFont font_16;
extern aFont font_30;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local routines
 */

static void sSetPixel(
  aGDOS* pGDOS, 
  aUIPixelType x, 
  aUIPixelType y
);

static void sSetLine(
  aGDOS* pGDOS, 
  aUIPixelType x1, 
  aUIPixelType y1,
  aUIPixelType x2, 
  aUIPixelType y2
);

static aFont* sMatchFontSize(
  const int size);

static aErr sGDOS_GetMemory(
  aUI* pUI, 
  void* vpData,
  char** ppMemory);

static aErr sGDOS_StartDrawing(
  aUI* pUI, 
  void* vpData);

static aErr sGDOS_Erase(
  aUI* pUI, 
  aGDRef gdRef);

static aErr sGDOS_TextWidth(
  aUI* pUI, 
  void* vpData,
  const char* pText,
  const unsigned int nSize,
  unsigned int* pnWidth);

static aErr sGDOS_SetColor(
  aUI* pUI, 
  void* vpData,
  aGDP* pSetColorPrimitive);

static aErr sGDOS_Line(
  aUI* pUI, 
  void* vpData,
  aGDP* pSetLinePrimitive);

static aErr sGDOS_Rect(
  aUI* pUI, 
  void* vpData,
  aGDP* pRectPrimitive);

static aErr sGDOS_Bezier(
  aUI* pUI, 
  void* vpData,
  aGDP* pArcPrimitive);

static aErr sGDOS_Text(
  aUI* pUI, 
  void* vpData,
  aGDP* pTextPrimitive);

static aErr sGDOS_Copy(
  aUI* pUI, 
  void* vpData,
  aGDP* pCopyPrimitive);

static aErr sGDOS_GetPixels(
  aUI* pUI,
  void* vpData,
  const aRECT* pSrcRect,
  char* pPixels);

static aErr sGDOS_EndDrawing(
  aUI* pUI, 
  void* vpData);

static aErr sGDOS_Destroy(
  aUI* pUI, 
  void* vpData);

static void sInterpolateImage(
  aGDOS* pSrc, 
  aGDOS* pDst,
  aRECT* dest);

static void sInterpolatePixel(
  unsigned char* dest, 
  aGDOS* pSrc, 
  float left, 
  float right,
  float top,
  float bottom);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSetPixel
 */

void sSetPixel(
  aGDOS* pGDOS, 
  aUIPixelType x, 
  aUIPixelType y
)
{
  char* p;

  /* bounds checking */
  if ((x < 0) || (x >= (aUIPixelType)pGDOS->nWidth))
    return;
  if ((y < 0) || (x >= (aUIPixelType)pGDOS->nWidth))
    return;

  p = pGDOS->pPixels + (((pGDOS->nWidth * y) + x) * 3);

  aMemCopy(p, pGDOS->rgb, 3);
  
} /* sSetPixel */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSetLine
 */

void sSetLine (
  aGDOS* pGDOS, 
  aUIPixelType x1, 
  aUIPixelType y1,
  aUIPixelType x2, 
  aUIPixelType y2
)
{
  aUIPixelType x, y;

  /* horizontal line */
  if (y1 == y2) {
    y = y1;
    if (x1 < x2) {
      for (x = x1; x <= x2; x++)
        sSetPixel(pGDOS, x, y);
    } else {
      for (x = x2; x <= x1; x++)
        sSetPixel(pGDOS, x, y);
    }

  /* vertical line */
  } else if (x1 == x2) {
    x = x1;
    if (y1 < y2) {
      for (y = y1; y <= y2; y++)
        sSetPixel(pGDOS, x, y);
    } else {
      for (y = y2; y <= y1; y++)
        sSetPixel(pGDOS, x, y);
    }

  /* line with defined slope */
  } else {
    float m;
    float r;
    m = ((float)(y2 - y1) / (float)(x2 - x1));
    if (m > 1) {
      if (x1 < x2) {
        r = y1;
        for (x = x1; x < x2; x++) {
          y = (aUIPixelType)(r + 0.5f);
          sSetPixel(pGDOS, x, y);
          r += m;
        }
      } else {
        r = y2;
        for (x = x2; x < x1; x++) {
          y = (aUIPixelType)(r + 0.5f);
          sSetPixel(pGDOS, x, y);
          r += m;
        }
      }
    } else {
      if (x1 < x2) {
        r = y1;
        for (x = x1; x < x2; x++) {
          y = (aUIPixelType)(r + 0.5f);
          sSetPixel(pGDOS, x, y);
          r += m;
        }
      } else {
        r = y2;
        for (x = x2; x < x1; x++) {
          y = (aUIPixelType)(r + 0.5f);
          sSetPixel(pGDOS, x, y);
          r += m;
        }
      }
    }
  }

} /* sSetLine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sMatchFontSize
 */

static aFont* sMatchFontSize(
  const int size) 
{
  switch (size) {
  case  8:  return &font_8;
  case  9:  return &font_9;
  case 10:  return &font_10;
  case 11:  return &font_11;
  case 12:  return &font_12;
  case 14:  return &font_14;
  case 16:  return &font_16;
  case 30:  return &font_30;
  } /* switch */

  return NULL;
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_CreateOffscreen
 */

aLIBRETURN 
aGD_CreateOffscreen(aUILib uiRef,
		    const aUIPixelType width,
		    const aUIPixelType height,
		    aGDRef* pGDRef,
		    aErr* pErr)
{
  return aGDInternal_CreateOffscreen((aUI*)uiRef, 
			             width,
			             height,
			             pGDRef,
			             pErr);

} /* aGD_CreateOffscreen */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGDInternal_CreateOffscreen
 *
 * Drawing that happens strictly in memory.  This is entirely cross-
 * platform.
 */

aLIBRETURN aGDInternal_CreateOffscreen(
  aUI* pUI,
  const aUIPixelType width,
  const aUIPixelType height,
  aGDRef* pGDRef,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)NULL;
  aGDOS* pGDOS = (aGDOS*)NULL;
  char*	pPixels = (char*)NULL;

  aVALIDUI(pUI);

  if ((uiErr == aErrNone) && !pGDRef)
    uiErr = aErrParam;
  if ((uiErr == aErrNone) && ((width < 1) || (height < 1)))
    uiErr = aErrRange;
    
  if (pGDRef)
    *pGDRef = 0;

  /* try to allocate the pixel storage */
  if (uiErr == aErrNone) {
    aMemSize pixelDataSize = sizeof(char) * 3 * width * height;
    pPixels = (char*)aMemAlloc(pixelDataSize);
    if (pPixels) {
      aBZero(pPixels, (aMemSize)pixelDataSize);
    } else
      uiErr = aErrMemory;
  }

  /* allocate the offscreen data */
  if (uiErr == aErrNone) {
    pGDOS = (aGDOS*)aMemAlloc(sizeof(aGDOS));
    if (pGDOS) {
      aBZero(pGDOS, sizeof(aGDOS));
      pGDOS->nHeight = (unsigned int)height;
      pGDOS->nWidth = (unsigned int)width;
      pGDOS->pPixels = pPixels;
    } else
      uiErr = aErrMemory;
  }
  
  /* allocate the GD */
  if (uiErr == aErrNone) {
    aRECT r;
    r.x = 0;
    r.y = 0;
    r.width = width;
    r.height = height;  
    uiErr = aGDShared_Create(pUI, pGDOS, &r, &pGD);
  }

  /* set up the offscreen drawing routines for this GD */
  if (uiErr == aErrNone) {
    pGDOS->pGD = pGD;
    pGD->getMemory    = sGDOS_GetMemory;
    pGD->startDrawing = sGDOS_StartDrawing;
    pGD->erase        = sGDOS_Erase;
    pGD->textWidth    = sGDOS_TextWidth;
    pGD->setColor     = sGDOS_SetColor;
    pGD->line         = sGDOS_Line;
    pGD->rect         = sGDOS_Rect;
    pGD->bezier       = sGDOS_Bezier;
    pGD->text         = sGDOS_Text;
    pGD->copy         = sGDOS_Copy;
#ifndef aPALM
    pGD->getPixels    = sGDOS_GetPixels;
#endif /* aPALM */
    pGD->endDrawing   = sGDOS_EndDrawing;
    pGD->destroy      = sGDOS_Destroy;
  }

  /* erase to start with a clean window */
  if (uiErr == aErrNone)
    uiErr = sGDOS_Erase(pUI, pGDOS);

  /* clean up if we failed */
  if (uiErr == aErrNone) {
    *pGDRef = pGD;
  } else {
    sGDOS_Destroy(pUI, pGD);
    if (pGD)
      aGDShared_Destroy(pGD);
  }

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGDInternal_CreateOffscreen */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDOS_GetMemory(
  aUI* pUI, 
  void* vpData,
  char** ppMemory
)
{
  aErr gdosErr = aErrNone;
  aGDOS* pOSGD = (aGDOS*)vpData;

  aAssert(ppMemory);

  *ppMemory = pOSGD->pPixels;

  return gdosErr;

} /* sGDOS_GetMemory */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDOS_StartDrawing(
  aUI* pUI, 
  void* vpData
)
{
  aErr gdosErr = aErrNone;

  return gdosErr;

} /* sGDOS_StartDrawing */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDOS_Erase(
  aUI* pUI,
  void* vpData
)
{
  aErr gdosErr = aErrNone;
  aGDOS* pOSGD = (aGDOS*)vpData;
  unsigned int i;
  unsigned int nPixels = pOSGD->nWidth * pOSGD->nHeight;
  char* p = pOSGD->pPixels;

  for (i = 0; i < nPixels; i++) {
    *p++ = (char)0xFF; /* red */
    *p++ = (char)0xFF; /* green */
    *p++ = (char)0xFF; /* blue */
  }

  return gdosErr;

} /* sGDOS_Erase */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDOS_TextWidth(
  aUI* pUI, 
  void* vpData,
  const char* pText,
  const unsigned int nSize,
  unsigned int* pnWidth) 
{
  aErr gdosErr = aErrNone;
  
  aFont* f = sMatchFontSize(nSize);
  
  *pnWidth = 0;

  if (f) {
    const char *p = pText;
    int width = 0;
    while (*p) {
      int idx = f->glyph[(int)*p];
      width += f->pGlyphs[idx * f->nGlyphSize];
      p++;
    }
    *pnWidth = width - 1; /* account for leading space */
  } else {
    gdosErr = aErrRange;
  }

  return gdosErr;

} /* sGDOS_TextWidth */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDOS_SetColor(
  aUI* pUI, 
  void* vpData,
  aGDP* pSetColorPrimitive
)
{
  aErr gdosErr = aErrNone;
  aGDOS* pOSGD = (aGDOS*)vpData;

  aAssert(pSetColorPrimitive->eType == kGDPSetColor);

  /* cache the color in pixel organized memory format for quick 
   * copying to the pixel buffer */
  pOSGD->rgb[0] = 
  	(char)((pSetColorPrimitive->f.setColor.color >> 16) & 0xFF);
  pOSGD->rgb[1] = 
  	(char)((pSetColorPrimitive->f.setColor.color >> 8) & 0xFF);
  pOSGD->rgb[2] = 
  	(char)(pSetColorPrimitive->f.setColor.color & 0xFF);

  return gdosErr;

} /* sGDOS_SetColor */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sGDOS_Line
 */

aErr sGDOS_Line(
  aUI* pUI, 
  void* vpData,
  aGDP* pSetLinePrimitive
)
{
  aErr gdosErr = aErrNone;
  unsigned int n, e;
  aGDOS* pOSGD = (aGDOS*)vpData;
  aPT* p;

  aAssert(pSetLinePrimitive->eType == kGDPLine);
  
  n = 0;
  e = pSetLinePrimitive->f.line.nPoints;
  p = pSetLinePrimitive->f.line.pPoints;
  aAssert(e >= 2);
  while (n < e - 1) {
    sSetLine(pOSGD, p[n].x, p[n].y, p[n + 1].x, p[n + 1].y);
    n++;
  }

  return gdosErr;

} /* sGDOS_Line */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDOS_Rect(
  aUI* pUI, 
  void* vpData,
  aGDP* pRectPrimitive
)
{
  aErr gdosErr = aErrNone;
  aUIPixelType x, y, xend, yend;
  aGDOS* pOSGD = (aGDOS*)vpData;

  aAssert(pRectPrimitive->eType == kGDPRect);
    
  if (pRectPrimitive->f.rect.bFilled) {
    x = pRectPrimitive->f.rect.bounds.x;
    y = pRectPrimitive->f.rect.bounds.y;
    xend = (aUIPixelType)(x + pRectPrimitive->f.rect.bounds.width - 1);
    yend = (aUIPixelType)(y + pRectPrimitive->f.rect.bounds.height - 1);
    while (y <= yend) {
      sSetLine(pOSGD, x, y, xend, y);
      y++;
    }

  } else {

    /* top line */
    x = pRectPrimitive->f.rect.bounds.x;
    y = pRectPrimitive->f.rect.bounds.y;
    xend = (aUIPixelType)(x + pRectPrimitive->f.rect.bounds.width - 1);
    sSetLine(pOSGD, x, y, xend, y);

    /* bottom line */
    y = (aUIPixelType)(y + pRectPrimitive->f.rect.bounds.height - 1);
    sSetLine(pOSGD, x, y, xend, y);

    /* left line */
    x = pRectPrimitive->f.rect.bounds.x;
    y = pRectPrimitive->f.rect.bounds.y;
    yend = (aUIPixelType)(y + pRectPrimitive->f.rect.bounds.height - 1);
    sSetLine(pOSGD, x, y, x, yend);

    /* right line */
    x = (aUIPixelType)(x + pRectPrimitive->f.rect.bounds.width - 1);
    sSetLine(pOSGD, x, y, x, yend);
  }

  return gdosErr;

} /* sGDOS_Rect */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDOS_Bezier(
  aUI* pUI, 
  void* vpData,
  aGDP* pArcPrimitive
)
{
  aErr gdosErr = aErrNone;
  
  return gdosErr;

} /* sGDOS_Arc */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDOS_Text(
  aUI* pUI, 
  void* vpData,
  aGDP* pTextPrimitive
)
{
  aErr gdosErr = aErrNone;
  aGDOS* pOSGD = (aGDOS*)vpData;
  int size = pTextPrimitive->f.text.size;
  aFont* f = sMatchFontSize(size);

  aAssert(pTextPrimitive->eType == kGDPText); 

  if (f) {
    char *p = pTextPrimitive->f.text.pText;
  
    /* holds the current character starting horizontal position */
    int x = pTextPrimitive->f.text.bounds.x - 1; /* account for the space */
    if (pTextPrimitive->f.text.flags & aUIALIGNRIGHT) {
      unsigned int w;
      gdosErr = sGDOS_TextWidth(pUI, vpData, p, (unsigned int)size, &w); 
      x += pTextPrimitive->f.text.bounds.width - w - 1;
    }
    if (pTextPrimitive->f.text.flags & aUIALIGNCENTER) {
      unsigned int w;
      gdosErr = sGDOS_TextWidth(pUI, vpData, p, (unsigned int)size, &w); 
      x += (pTextPrimitive->f.text.bounds.width - w) / 2;
    }
 
    if (gdosErr == aErrNone) {
    	
      /* compute the top of the bounds */
      int y = pTextPrimitive->f.text.bounds.y 
    	      + (pTextPrimitive->f.text.bounds.height - size) / 2;

      int span = f->rowsize;

      while (*p) {
	int i;
        int idx = f->glyph[(int)*p];
        int width = f->pGlyphs[idx * f->nGlyphSize];
        unsigned char* bitmap = &f->pGlyphs[idx * f->nGlyphSize + 1];

        /* for each glyph row */
        for (i = 0; i < f->rows; i++, bitmap += span) {
          unsigned char mask = 0x80;
          unsigned char* b = bitmap;
          unsigned char c = *b;
	  int j;
          for (j = 0; j < width; j++) {
            if (mask == 0) {
              b++;
              c = *b;
              mask = 0x80;
            }
            if (c & mask)
              sSetPixel(pOSGD, (aUIPixelType)(x + j), (aUIPixelType)(y + i));
            mask >>= 1;
          }
      	
        }

        /* move to the next character */
        x += width;
        p++;
      }
    }
  } else {
    gdosErr = aErrRange;
  }

  return gdosErr;

} /* sGDOS_Text */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGDOS_Copy
 *
 * Assumes the region being copied to is not to be clipped.
 */

aErr sGDOS_Copy(
  aUI* pUI, 
  void* vpData,
  aGDP* pCopyPrimitive
)
{
  aErr gdosErr = aErrNone;
  aGDOS* pOSGD = (aGDOS*)vpData;
  aRECT dest = pCopyPrimitive->f.copy.dest;
  aGD* pSrcGD = (aGD*)pCopyPrimitive->f.copy.pCopyGD;
  aGDOS* pSource = (aGDOS*)pSrcGD->vpData;
  unsigned int y;

  aAssert(pCopyPrimitive->eType == kGDPCopy);

  /* our source is an offscreen, just like us */
  if (((unsigned int)dest.width == pSource->nWidth) 
      && ((unsigned int)dest.height == pSource->nHeight)) {

    /* the simple case is where the source and destination are the 
     * same size */
    for (y = 0; y < pSource->nHeight; y++) {
      /* compute the memory locations for both source and dest 
       * row start */
      char* s = &pSource->pPixels[y * pSource->nWidth * 3];
      char* d = &pOSGD->pPixels[(dest.x 
    			       + (dest.y + y) * pOSGD->nWidth) * 3];
      aMemCopy(d, s, pSource->nWidth * 3);
    }

  } else {

    /* we need to interpolate the image */
    sInterpolateImage(pSource, pOSGD, &dest);
  }

  return gdosErr;
}


#ifndef aPALM
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sGDOS_GetPixels
 *
 * retrieve the pixels from the offscreen into a buffer assumed to
 * be the same size as the source rectangle passed in.
 */

aErr sGDOS_GetPixels(
  aUI* pUI,
  void* vpData,
  const aRECT* pSrcRect,
  char* pPixels
)
{
  aErr gdosErr = aErrNone;
  aGDOS* pGDOS = (aGDOS*)vpData;
  aUIPixelType x, y, width, height;
  aUIPixelType temp;
  int rowSize = pGDOS->nWidth * 3; /* for RGB */
  int copyRowSize;
  char* s;
  char* d;

  /* figure out a legal window in the offscreen */
  x = (pSrcRect->x < 0) ? 0 : pSrcRect->x;
  y = (pSrcRect->y < 0) ? 0 : pSrcRect->y;
  temp = (aUIPixelType)(pSrcRect->x + pSrcRect->width);
  width = (temp < (aUIPixelType)pGDOS->nWidth) ? 
		    (aUIPixelType)(temp - x) : 
		    (aUIPixelType)(pGDOS->nWidth - x);
  temp = (aUIPixelType)(pSrcRect->y + pSrcRect->height);
  height = (temp < (aUIPixelType)pGDOS->nHeight) ? 
		    (aUIPixelType)(temp - y) : 
		    (aUIPixelType)(pGDOS->nHeight - y);

  /* point to the first pixel to copy */
  s = pGDOS->pPixels + (((y * pGDOS->nWidth) + x) * 3);
  d = pPixels;

  /* for each row, copy over the data */
  copyRowSize = width * 3; /* for RGB */
  for (temp = 0; temp < height; temp++) {
    aMemCopy(d, s, copyRowSize);
    s += rowSize;
    d += copyRowSize;
  }

  return gdosErr;

} /* sGDOS_GetPixels */
#endif /* aPALM */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDOS_EndDrawing(
  aUI* pUI, 
  void* vpData
)
{
  aErr gdosErr = aErrNone;
  aGDOS* pGDOS = (aGDOS*)vpData;

  if (gdosErr == aErrNone)
    gdosErr = aGDShared_IssueDrawList(pGDOS->pGD);

  return gdosErr;

} /* sGDOS_EndDrawing */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sGDOS_Destroy(
  aUI* pUI, 
  void* vpData
)
{
  aErr gdosErr = aErrNone;
  aGDOS* pGDOS = (aGDOS*)vpData;

  if (pGDOS) {
    if (pGDOS->pPixels) {
      aMemFree(pGDOS->pPixels);
      pGDOS->pPixels = (char*)NULL;
    }
    aMemFree(pGDOS);
  }

  return gdosErr;

} /* sGDOS_Destroy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sInterpolateImage
 *
 * This routine does a simple interpolation to scale the source image
 * to fit the destination image.
 *
 * In the case of an image becoming smaller, the resulting image pixels
 * are an average by distance from the src pixel region
 * 
 *  s00     s10     s20     s30
 *       --------------
 *       |            | 
 *  s10  |  s11     s21     s31
 *       |            |
 *       -------------- 
 *  s20     s21     s22     s32
 * 
 * We figure out how much area each source pixel covers in the destination
 * pixel (normalized the units of the destination) and then we average based
 * on the resulting weights.
 * 
 * If the source image is 6x5 and the destination is 2x2 the boundaries of 
 * the destination pixels within the source bitmap are:
 * 
 * 0.0,0.0-----------3.0,0.0-----------6.0,0.0
 *    |                 |                 |
 *    |   area = 7.5    |                 |
 *    |                 |                 |
 * 0.0,2.5-----------3.0,2.5-----------6.0,2.5
 *    |                 |                 |
 *    |                 |                 |
 *    |                 |                 |
 * 0.0,5.0-----------3.0,5.0-----------6.0,5.0
 *
 * So, the destination pixel sizes are src/dest width, src/dest height. 
 */

void sInterpolateImage(
  aGDOS* pSrc, 
  aGDOS* pDst,
  aRECT* pBounds)
{
  float Xinc = (float)pSrc->nWidth / (float)pBounds->width;
  float Yinc = (float)pSrc->nHeight / (float)pBounds->height;
  float x1;
  float x2;
  float y1 = 0;
  float y2 = Yinc;
  int x, y;

  for (y = 0; y < pBounds->height; y++, y1 += Yinc, y2 += Yinc) {
    unsigned char* d = (unsigned char*)&pDst->pPixels[
             (pBounds->x + (pBounds->y + y) * pDst->nWidth) * 3];
    x1 = 0;
    x2 = Xinc;
    for (x = 0; x < pBounds->width; x++, x1 += Xinc, x2 += Xinc) {
      sInterpolatePixel(d, pSrc, x1, x2, y1, y2);
      d += 3;
    } /* for x */
  } /* for y */
} /* sInterpolateImage */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sInterpolateImage
 */

void sInterpolatePixel(
  unsigned char* dest, 
  aGDOS* pSrc, 
  float left, 
  float right,
  float top,
  float bottom)
{
  float fdR = 0.0;
  float fdG = 0.0;
  float fdB = 0.0;
  float x, y;
  int iX, iY;
  float aX, aY;
  float area;
  unsigned char* s;
  float totalArea = (right - left) * (bottom - top);

  y = top;
  iX = (int)left;
  while (y < bottom) {

    iY = (int)y;
    s = (unsigned char*)&pSrc->pPixels[(iX + iY * pSrc->nWidth) * 3];

    /* compute our increment in Y */
    if (y != (int)y) {
      aY = ((int)(y + 1)) - y;
    } else {
      aY = 1.0f;
      if (y + aY > bottom)
	aY = bottom - y;
    }
    y += aY;

    x = left;
    while (x < right) {

      /* compute our increment in X */
      if (x != (int)x) {
        aX = ((int)(x + 1)) - x;
      } else {
	aX = 1.0f;
	if (x + aX > right)
	  aX = right - x;
      }
      x += aX;

      /* contribute the source pixel based on it's area in the destination */
      area = aX * aY;
      fdR += area * *s++;
      fdG += area * *s++;
      fdB += area * *s++;

    } /* while x */
      
  } /* while y */

  *dest++ = (unsigned char)(fdR / totalArea + 0.5f);
  *dest++ = (unsigned char)(fdG / totalArea + 0.5f);
  *dest = (unsigned char)(fdB / totalArea + 0.5f);

} /* sInterpolatePixel */
