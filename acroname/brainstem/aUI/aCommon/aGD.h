/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aGD.h	                                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent graphics      */
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

#ifndef _aGD_H_
#define _aGD_H_

#include "aUIInternal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* graphics device callbacks */
typedef aErr (*gdResize)(aUI* pUI, 
			 void* vpData);
typedef aErr (*gdStartDrawing)(aUI* pUI, 
			       void* vpData);
typedef aErr (*gdErase)(aUI* pUI, 
			void* vpData);
typedef aErr (*gdTextWidth)(aUI* pUI, 
                            void* vpData,
                            const char* pText,
                            const unsigned int nSize,
                            unsigned int* pnWidth);
typedef aErr (*gdGetPixels)(aUI* pUI, 
		            void* vpData,
		            const aRECT* pSrcRect,
		            char* pPixels);
typedef aErr (*gdEndDrawing)(aUI* pUI, 
			     void* vpData);
typedef aErr (*gdDestroy)(aUI* pUI, 
			  void* vpData);
typedef aErr (*gdGetMemory)(aUI* pUI,
			    void* vpData,
			    char** ppMemory);

#define kGDPSetColor	0x01
#define kGDPLine	0x02
#define kGDPRect	0x03
#define kGDPElipse	0x04
#define kGDPSetFont     0x05
#define kGDPText	0x06
#define kGDPBezier	0x07
#define kGDPCopy	0x08

typedef struct aGDP* aGDPPtr;

typedef aErr (*gdPrimitiveProc)(aUI* pUI,
				void* vpData,
		                aGDPPtr pPrimitive);
typedef aErr (*gdPrimitiveCleanupProc)(aGDPPtr pPrimitive);

typedef struct aGDP {

  unsigned char			eType;

  /* primitive specifics */
  union {
    struct {
      unsigned long 		color;
    } setColor;

    struct {
      aPT*			pPoints;
      unsigned int		nPoints;
    } line;

    struct {
      aRECT			bounds;
      aBool			bFilled;
    } rect;

    struct {
      aRECT			bounds;
      aBool			bFilled;
    } elipse;

    struct {
      aFONTDEF                  def;
    } textdef;

    struct {
      aRECT			bounds;
      int			flags;
      unsigned int		size;
      char*			pText;
    } text;

    struct {
      aPT			points[4];
    } bezier;

    struct {
      aRECT                     dest;
      struct aGD*		pCopyGD;
    } copy;

  } f;

  /* link to next primitive */
  aGDPPtr			pNext;

  /* callbacks (methods) */  
  gdPrimitiveProc		pCall;
  gdPrimitiveCleanupProc	pCleanup;

  /* GD for font manipulation */
  struct aGD*		       	pGD;

} aGDP;


typedef struct aGD {
  /* OS data */
  void*			vpData;

  /* shared data */
  aRECT			r;

  /* callback procedures */
  gdGetMemory           getMemory;
  gdResize		resize;
  gdStartDrawing	startDrawing;
  gdErase		erase;
  gdTextWidth		textWidth;
  gdPrimitiveProc	setColor;
  gdPrimitiveProc	line;
  gdPrimitiveProc	rect;
  gdPrimitiveProc	elipse;
  gdPrimitiveProc	bezier;
  gdPrimitiveProc       textdef; /* aGD sets this, subclasses needn't */
  gdPrimitiveProc	text;
  gdPrimitiveProc       copy;
  gdGetPixels		getPixels;
  gdEndDrawing		endDrawing;
  gdDestroy		destroy;

  /* state */
  aFONTDEF		m_font;

  /* draw list */
  struct aUI*		pUI;
  aGDP*			pDrawlist;
  aGDP*			pDrawlast;

  /* safety check */
  int			check;
} aGD;

#ifdef __cplusplus 
}
#endif


#define aGDCHECK	(int)0x12321

#define aVALIDGD(p)						   \
  if ((p == NULL) ||						   \
      (((aGD*)p)->check != aGDCHECK)) {		   		   \
    uiErr = aErrParam;						   \
  }



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Functions shared by screen and off-screen GD's.
 */

#ifdef __cplusplus
extern "C" {
#endif

aErr aGDShared_Create(struct aUI* pUI,
		      void* vpData,
		      const aRECT* pRect,
		      aGD** ppGD);
aErr aGDShared_Destroy(aGD* pGD);

aErr aGDShared_IssueDrawList(aGD* pGD);


#ifdef __cplusplus
}
#endif

#endif /* _aGD_H_ */

