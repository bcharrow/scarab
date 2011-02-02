/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aUI.h	                                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent user 	   */
/*		interface layer.       				   */
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

#ifndef _aUI_H_
#define _aUI_H_

#include "aIO.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * opaque reference to the UI library
 */

typedef aLIBREF aUILib;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * define symbol import mechanism
 */

#ifndef aUI_EXPORT
#define aUI_EXPORT aLIB_IMPORT
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * UI library manipulation routines
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

aUI_EXPORT aLIBRETURN 
aUI_GetLibRef(aUILib* pUIRef, 
	      aErr* pErr);

aUI_EXPORT aLIBRETURN 
aUI_ReleaseLibRef(aUILib UIRef, 
		  aErr* pErr);

aUI_EXPORT aLIBRETURN 
aUI_GetVersion(aUILib UIRef, 
	       unsigned long* pVersUIn,
	       aErr* pErr);

#ifdef __cplusplus 
}
#endif /* __cplusplus */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Common Dialogs
 */

typedef aErr (*aUIDialogIdleProc)(const void* ref);
typedef aBool (*aUIPickFileFilterProc)(const char* pFilename,
				       const unsigned long nSize);

#ifdef __cplusplus
extern "C" {
#endif

aUI_EXPORT aLIBRETURN 
aDialog_Message(aUILib uiRef,
		const char* pMessage,
		aUIDialogIdleProc idleProc,
		void* idleRef,
		aErr* pErr);

aUI_EXPORT aLIBRETURN 
aDialog_PickFile(aUILib uiRef,
		 const char* pMessage,
		 char* pFileName,
		 const aFileArea eFileArea,
		 aUIPickFileFilterProc filterProc,
		 aUIDialogIdleProc idleProc,
		 void* idleRef,
		 aErr* pErr);

#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Graphics Device definitions
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef void* aGDRef;

typedef short aUIPixelType;
typedef float aUIAngleType;

typedef struct aPT {
  aUIPixelType x;
  aUIPixelType y;
} aPT;

typedef struct aRECT {
  aUIPixelType x;
  aUIPixelType y;
  aUIPixelType width;
  aUIPixelType height;
} aRECT;

#ifdef aUNIX
typedef struct aUnixGDCreateData {
  void* pDisplay;
  int parent;
} aUnixGDCreateData;
#endif

#define aUIDEFAULTFONTSIZE	12
#define aUIALIGNCENTER		0x01
#define aUIALIGNLEFT		0x02
#define aUIALIGNRIGHT		0x04
#define aUIALIGNMASK		0x07
#define aUITEXTINVISIBLE	0x80
#define aUIFIXEDWIDTHFONT       0x01

typedef struct aFONTDEF {
  char flags;
} aFONTDEF;


aUI_EXPORT aLIBRETURN 
aGD_Create(aUILib uiRef,
	   const aRECT* pRect, 
	   void* createData,
	   aGDRef* pGDRef,
	   aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_CreateOffscreen(aUILib uiRef,
		    const aUIPixelType width,
		    const aUIPixelType height,
		    aGDRef* pGDRef,
		    aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_GetGraphicsMemory(aUILib uiRef,
		      aGDRef gdRef,
		      char** ppMemory,
		      aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_CreatePDF(aUILib uiRef,
	      const char* pFileName,
	      const aFileArea eFileArea,
	      aGDRef* pGDRef,
	      aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_GetSize(aUILib uiRef,
	    aGDRef gdRef,
	    aRECT* pRect, 
	    aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_SetSize(aUILib uiRef,
	    aGDRef gdRef,
	    const aRECT* pRect, 
	    aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_StartDrawing(aUILib uiRef,
		 aGDRef gdRef,
		 aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_Erase(aUILib uiRef,
	  aGDRef gdRef,
	  aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_SetColor(aUILib uiRef,
	     aGDRef gdRef,
	     const unsigned long color,
	     aErr* pErr);

aUI_EXPORT aLIBRETURN
aGD_Line(aUILib uiRef,
	 aGDRef gdRef,
	 const aPT* pPoints,
	 const unsigned int nPoints,
	 aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_Rect(aUILib uiRef,
	 aGDRef gdRef,
	 const aRECT* pRect,
	 const aBool bFilled,
	 aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_Elipse(aUILib uiRef,
	   aGDRef gdRef,
	   const aRECT* pRect,
	   const aBool bFilled,
	   aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_RoundedRect(aUILib uiRef,
		aGDRef gdRef,
		const aRECT* pRect,
		const aUIPixelType radius,
		aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_Arc(aUILib uiRef,
	aGDRef gdRef,
	const aPT* pCenter,
	const aUIPixelType radius,
	const aUIAngleType startAngle,
	const aUIAngleType endAngle,
	aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_SetFont(aUILib uiRef,
	    aGDRef gdRef,
	    const aFONTDEF* pFontDef,
	    aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_Text(aUILib uiRef,
	 aGDRef gdRef,
	 const aRECT* pRect,
	 const int flags,
	 const unsigned int nSize,
	 const char* pText,
	 aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_Paragraph(aUILib uiRef,
	      aGDRef gdRef,
	      const aRECT* pRect,
	      const int flags,
	      const unsigned int nSize,
	      char** ppText,
	      unsigned int* pnLines,
	      aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_Copy(aUILib uiRef,
	 aGDRef gdRef,
	 const aRECT* pDstRect,
	 aGDRef srcGDRef,
	 const aRECT* pSrcRect,
	 aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_EndDrawing(aUILib uiRef,
	       aGDRef gdRef,
	       aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_ReadJPG(aUILib uiRef,
	    aStreamRef jpgStream,
	    aGDRef* pGDRef,
	    aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_WritePNG(aUILib uiRef,
	     aGDRef gdRef,
	     aStreamRef pngStream,
	     aErr* pErr);

aUI_EXPORT aLIBRETURN 
aGD_TextWidth(aUILib uiRef,
	      aGDRef gdRef,
	      const char* pText,
	      const unsigned int nSize,
	      unsigned int* pnWidth,
	      aErr* pErr);

aUI_EXPORT aLIBRETURN
aGD_Destroy(aUILib uiRef,
	    aGDRef gdRef,
	    aErr* pErr);

#if defined(aUNIX) && !defined(aMACX)
aUI_EXPORT aLIBRETURN 
aGD_HandleEvent(aUILib uiRef,
		void* pXEvent,
		aBool* pbHandled,
		aErr *pErr);
#endif /* aUNIX && !aMACX */

#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * HTTP Definitions
 */

#define aHTTPMAXREQESTLINE	80
#define aHTTPMAXTEMPLATELINE	200
#define aHTTPMAXPARAMLEN	aMAXIDENTIFIERLEN
#define aHTTPSETTINGFILE        "http.config"
#define aHTTPSETTINGMAX         32
#define aHTTPPORTKEY            "http-port"
#define aHTTPPORTDEFAULT        8000
#define aHTTPADDRKEY            "http-address"
#define aHTTPADDRDEFAULT        0x7F000001

typedef void* aHTTPRef;

typedef aErr (*aHTTPRequestProc)(const char* pURLStr,
				 aSymbolTableRef params,
				 aStreamRef reply,
				 void* vpRef);
typedef aErr (*aHTTPTemplateProc)(const unsigned int nParamIndex,
				  const unsigned int nBlockIndex,
				  aStreamRef reply,
				  void* vpRef);

#ifdef __cplusplus
extern "C" {
#endif

aUI_EXPORT aLIBRETURN 
aHTTP_Create(aUILib uiRef,
	     aSettingFileRef settings,
	     aHTTPRequestProc requestProc,
	     void* vpRef,
	     aHTTPRef* pHTTPRef,
	     aErr* pErr);

aUI_EXPORT aLIBRETURN 
aHTTP_TimeSlice(aUILib uiRef,
		aHTTPRef http,
		aBool* bChanged,
		aErr* pErr);

aUI_EXPORT aLIBRETURN 
aHTTP_Template(aUILib uiRef,
	       aHTTPRef http,
	       const char* pNameStr,
	       aHTTPTemplateProc templateProc,
	       void* vpRef,
	       aStreamRef reply,
	       aErr* pErr);

aUI_EXPORT aLIBRETURN 
aHTTP_Destroy(aUILib uiRef,
	      aHTTPRef http,
	      aErr* pErr);

#ifdef __cplusplus 
}
#endif



#ifdef __cplusplus
extern "C" {
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Browser Definitions
 */

aUI_EXPORT aLIBRETURN 
aBrowser_LaunchURL(aUILib uiRef,
		   const char* pURLStr,
		   aErr* pErr);

#ifdef __cplusplus 
}
#endif



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Graphics Widget definitions
 */


#ifdef __cplusplus
extern "C" {
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Browser Definitions
 */

aUI_EXPORT aLIBRETURN 
aWidget_CreateLogView(aUILib uiRef,
		      void* createData,
		      const aRECT* pRect, 
		      const unsigned int nFontSize,
		      const unsigned int nMaxLines,
		      aStreamRef* pLogStream,
		      aErr* pErr);

#ifdef __cplusplus 
}
#endif

#endif /* _aUI_H_ */
