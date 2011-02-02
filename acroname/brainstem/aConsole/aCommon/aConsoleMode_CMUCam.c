/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsoleMode_CMUCam.c                                     */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent CMUCam    */
/*		mode callbacks.					   */
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

#ifdef aCMUCAM

#include "aUtil.h"
#include "aConsoleText.h"
#include "aCMUCamUtil.h"
#include "aCMUCam_TC.h"
#include "aConsoleMode_CMUCam.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sCameraURL(
  aConsole* pConsole,
  aSymbolTableRef params,
  aStreamRef reply
);
static aErr sOpURL(
  aConsole* pConsole,
  aSymbolTableRef params,
  aStreamRef reply
);

static aErr sProcessCameraLine (
  aConsole* pConsole,
  const char* line
);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCameraURL
 *
 * callback to allow handing of the main camera URL
 */

aErr sCameraURL(aConsole* pConsole,
		aSymbolTableRef params,
		aStreamRef reply)

{
  aErr cmuErr = aErrNone;

  /* Now we send out a HTML page from a template to actually
   * handle the request, The template allows insertion of 
   * state-dependent data so we will insert the current values
   * for x and y in this page.  The template page lives in
   * the aAsset directory and when a substitution character
   * is encountered in parsing the template, the sRenderCamera
   * callback is called to allow us to substitute the current
   * value. */
  if (cmuErr == aErrNone)
    aHTTP_Template(pConsole->uiLib, 
  		   pConsole->http,
  		   "cmuCamera.tpl",
  		   NULL,
  		   (void*)pConsole,
  		   reply,
  		   &cmuErr);

  return cmuErr;

} /* sCameraURL */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sOpURL
 *
 * callback to allow handing of the operation URL
 */

aErr sOpURL(
  aConsole* pConsole,
  aSymbolTableRef params,
  aStreamRef reply
)
{
  aErr cmuErr = aErrNone;
  aErr paramErr = aErrNone;
  int i;
  char buf[10];

  /* Here the params are the combination of the http GET and POST
   * parameters from any form submitted as part of this URL.
   * We search these for the threshold parameters and assign them
   * in the cmucam structure. */
  if (!params)
    paramErr = aErrIO;
  if (paramErr == aErrNone) {
    int val;
    buf[0] = 't';
    for (i = 0; (i < 6) && (paramErr == aErrNone); i++) {
      aStringFromInt(&buf[1], i);
      paramErr = aSymbolTable_GetInt(pConsole->ioLib, params, 
      				     buf, &val);
      if (paramErr == aErrNone) {
        if (val < 0) val = 0;
        if (val > 255) val = 255;
        pConsole->mode.cmucam.thresholds[i] = (unsigned char)val;
      }
    }
  }

  /* We can now initiate the track color */
  if (paramErr == aErrNone) {
    aStringCopy(pConsole->mode.cmucam.operation, "TC ");
    for (i = 0; i < 6; i++) {
      aStringFromInt(buf, pConsole->mode.cmucam.thresholds[i]);
      aStringCat(pConsole->mode.cmucam.operation, buf);
      if (i < 5)
        aStringCat(pConsole->mode.cmucam.operation, " ");
    }
    aStringCopy(pConsole->mode.cmucam.opName, "Tracking Color");
  }

  if ((cmuErr == aErrNone)
      && pConsole->mode.cmucam.operation[0]) {
    cmuErr = aConsoleMode_CMUCam_Line(pConsole, pConsole->mode.cmucam.operation);
    pConsole->mode.cmucam.operation[0] = 0;
  }

  if (cmuErr == aErrNone)
    aHTTP_Template(pConsole->uiLib, 
  		   pConsole->http,
  		   "cmuOp.tpl",
  		   aCMUCam_RenderOpHTML,
  		   (void*)pConsole,
  		   reply,
  		   &cmuErr);

  return cmuErr;

} /* sOpURL */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aCMUCam_RenderOpHTML
 *
 * This is the callback for HTML template subtitution parameters.
 */

aErr aCMUCam_RenderOpHTML (
  const unsigned int nParamIndex,
  const unsigned int nBlockIndex,
  aStreamRef reply,
  void* vpRef
)
{
  aErr cmuErr = aErrNone;
  aConsole* pConsole = (aConsole*)vpRef;
  char line[30];
  
  line[0] = 0;
 
  aAssert(pConsole);

  if ((nBlockIndex == 0) && (nParamIndex == 0)) {
    aStringCopy(line, pConsole->mode.cmucam.opName);
  }

  if (line[0])
    aStream_Write(aStreamLibRef(reply), reply, 
  		  line, aStringLen(line), &cmuErr); 

  return cmuErr;

} /* aCMUCam_RenderOpHTML */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sProcessCameraLine
 *
 * Process a line of text that the camera has sent.
 */

static aErr sProcessCameraLine (
  aConsole* pConsole,
  const char* line
)
{
  aErr consoleErr = aErrNone;
  char buf[100];
  char* p = buf;
  
  aStringCopy(buf, line);

  switch (*p) {

  case 'M':
    p += 2;
    pConsole->mode.cmucam.cursorX = aCMUCam_NextInt(&p);
    pConsole->mode.cmucam.cursorY = aCMUCam_NextInt(&p);
    pConsole->mode.cmucam.cursorLeft = aCMUCam_NextInt(&p);
    pConsole->mode.cmucam.cursorTop = aCMUCam_NextInt(&p);
    pConsole->mode.cmucam.cursorRight = aCMUCam_NextInt(&p);
    pConsole->mode.cmucam.cursorBottom = aCMUCam_NextInt(&p);
    aCMUCam_NextInt(&p);
    pConsole->mode.cmucam.cursorConfidence = aCMUCam_NextInt(&p);
    aConsoleMode_CMUCam_Draw(pConsole);
    break;

  } /* switch */
  
  return consoleErr;

} /* sProcessCameraLine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleMode_CMUCam_Init
 *
 * callback to set up any mode resources
 */

aErr aConsoleMode_CMUCam_Init(aConsole* pConsole)
{
  aErr cmuErr = aErrNone;

  if (cmuErr == aErrNone)
    cmuErr = aConsole_CreateDrawPane(pConsole, 
    				     DRAWPANEWIDTH, 
    				     DRAWPANEHEIGHT);

  if (cmuErr == aErrNone)
    cmuErr = aConsole_DisplayLine(pConsole, 
      				  aSTATUS_CMUCAM_MODE, 
      				  kDisplayStatus);

  /* set up some pleasing initial values */   
  if (cmuErr == aErrNone) {
    pConsole->mode.cmucam.thresholds[0] = 160;
    pConsole->mode.cmucam.thresholds[1] = 250;
    pConsole->mode.cmucam.thresholds[2] = 160;
    pConsole->mode.cmucam.thresholds[3] = 250;
    pConsole->mode.cmucam.thresholds[4] = 160;
    pConsole->mode.cmucam.thresholds[5] = 250;
  }

  return cmuErr;

} /* aConsoleMode_CMUCam_Init */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleMode_CMUCam_Cleanup
 *
 * callback to clean up any mode resources
 */

aErr aConsoleMode_CMUCam_Cleanup(aConsole* pConsole)
{
  aErr cmuErr = aErrNone;    

  if (cmuErr == aErrNone)
    cmuErr = aConsole_DestroyDrawPane(pConsole);

  return cmuErr;

} /* aConsoleMode_CMUCam_Cleanup */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleMode_CMUCam_Draw
 *
 * callback to redraw the drawing area in the UI
 */

aErr aConsoleMode_CMUCam_Draw(aConsole* pConsole)
{
  aErr cmuErr = aErrNone;
  aPT line[2];

  /* initiate the drawing of the drawing pane */  
  if (cmuErr == aErrNone)
    aGD_StartDrawing(pConsole->uiLib, pConsole->drawGD, &cmuErr);

  /* erase to start with (NULL rectangle means entire pane) */
  if (cmuErr == aErrNone)
    aGD_Erase(pConsole->uiLib, pConsole->drawGD, &cmuErr);
  
  /* set a color for the two diagonals (RGB) */
  if (cmuErr == aErrNone)
    aGD_SetColor(pConsole->uiLib, pConsole->drawGD,
    		 0x00FF00, &cmuErr);

  /* draw one diagonal */
  if (cmuErr == aErrNone) {
    line[0].x = 0;
    line[0].y = 0;
    line[1].x = (aUIPixelType)pConsole->drawWidth;
    line[1].y = (aUIPixelType)pConsole->drawHeight;
    aGD_Line(pConsole->uiLib, pConsole->drawGD,
    	     line, 2, &cmuErr);
  }

  /* draw the other diagonal */
  if (cmuErr == aErrNone) {
    line[0].x = 0;
    line[0].y = (aUIPixelType)pConsole->drawHeight;
    line[1].x = (aUIPixelType)pConsole->drawWidth;
    line[1].y = 0;
    aGD_Line(pConsole->uiLib, pConsole->drawGD,
    	     line, 2, &cmuErr);
  }

  /* set a color for the cursor (RGB) */
  if (cmuErr == aErrNone)
    aGD_SetColor(pConsole->uiLib, pConsole->drawGD,
    		 0xFF0000, &cmuErr);

  /* draw the cursor if it is present */
  if ((cmuErr == aErrNone)
      && pConsole->mode.cmucam.cursorX
      && pConsole->mode.cmucam.cursorX) {
    
    /* do a crosshair in red at the cursor x, y position */
    aUIPixelType x = aCMUCam_PaneX(pConsole->mode.cmucam.cursorX);
    aUIPixelType y = aCMUCam_PaneY(pConsole->mode.cmucam.cursorY);
    aRECT r;
    if (cmuErr == aErrNone) {
      line[0].x = x;
      line[0].y = 0;
      line[1].x = x;
      line[1].y = DRAWPANEHEIGHT - 1;
      aGD_Line(pConsole->uiLib, pConsole->drawGD,
    	       line, 2, &cmuErr);
    }
    if (cmuErr == aErrNone) {
      line[0].x = 0;
      line[0].y = y;
      line[1].x = DRAWPANEWIDTH - 1;
      line[1].y = y;
      aGD_Line(pConsole->uiLib, pConsole->drawGD,
    	       line, 2, &cmuErr);
    }

    /* set a color for bounds box */
    if (cmuErr == aErrNone)
      aGD_SetColor(pConsole->uiLib, pConsole->drawGD,
    		   0x000000, &cmuErr);

    /* draw the bounds box */
    if (cmuErr == aErrNone) {
      int height = aCMUCam_PaneY(pConsole->mode.cmucam.cursorTop) - 
      		   aCMUCam_PaneY(pConsole->mode.cmucam.cursorBottom);
      r.x = aCMUCam_PaneX(pConsole->mode.cmucam.cursorLeft);
      r.y = (aUIPixelType)
            (aCMUCam_PaneY(pConsole->mode.cmucam.cursorTop) - height);
      r.width = (aUIPixelType)
      		(aCMUCam_PaneX(pConsole->mode.cmucam.cursorRight) - r.x);
      r.height = (aUIPixelType)height;
      aGD_Rect(pConsole->uiLib, pConsole->drawGD,
    	       &r, aTrue, &cmuErr);
    }

    /* fill in the bounds with the confidence factor color */
    if (cmuErr == aErrNone) {
      int c = pConsole->mode.cmucam.cursorConfidence;
      unsigned long confidenceColor = (unsigned long)(c << 8);
      aGD_SetColor(pConsole->uiLib, pConsole->drawGD,
    		   confidenceColor, &cmuErr);
      r.x += 2;
      r.y += 2;
      r.width -= 4;
      r.height -= 4;
    }
    if (cmuErr == aErrNone)
      aGD_Rect(pConsole->uiLib, pConsole->drawGD,
    	       &r, aTrue, &cmuErr);
  }

  if (cmuErr == aErrNone)
    aGD_EndDrawing(pConsole->uiLib, pConsole->drawGD, &cmuErr);

  return cmuErr;

} /* aConsoleMode_CMUCam_Draw */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleMode_CMUCam_HTTP
 *
 * callback to allow handing of HTTP requests
 */

aErr aConsoleMode_CMUCam_HTTP(const char* pURL,
			      aSymbolTableRef params,
			      aStreamRef reply,
			      void* vpRef)
{
  aErr cmuErr = aErrNone;
  aConsole* pConsole = (aConsole*)vpRef;

  /* we handle the url's we care about here and return not found
   * if the url is one we don't recognize */  
  if (!aStringCompare(pURL, "/cmucam")
      || !aStringCompare(pURL, "/draw")) {
    cmuErr = sCameraURL(pConsole, params, reply);
  } else if (!aStringCompare(pURL, "/cmuTC")) {
    cmuErr = aCMUCam_TC(pConsole, params, reply);
  } else if (!aStringCompare(pURL, "/cmuOp")) {
    cmuErr = sOpURL(pConsole, params, reply);
  } else
    cmuErr = aErrNotFound;

  return cmuErr;

} /* aConsoleMode_CMUCam_HTTP */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleMode_CMUCam_Slice
 *
 * This time slice routine gets called periodically, Here we can 
 * check for information inbound from the camera.  We get the 
 * characters and then hand them to a display stream which dumps
 * the lines to the display when the line terminator is reached.
 */

aErr aConsoleMode_CMUCam_Slice(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;
  aErr readErr;
  char c;

  /* swallow up any available characters and send them to the 
   * display buffer */
  do {
    aStream_Read(pConsole->ioLib, 
    		 pConsole->linkStreamRef,
    		 &c, 1, &readErr);
    switch (readErr) {

    case aErrNone:
      if (c == '\r') {
        pConsole->mode.cmucam.input[pConsole->mode.cmucam.nInput] = 0;
        sProcessCameraLine(pConsole, pConsole->mode.cmucam.input);
        aConsole_DisplayLine(pConsole, pConsole->mode.cmucam.input,
        		     kDisplayInput);
        pConsole->mode.cmucam.nInput = 0;
      } else
        pConsole->mode.cmucam.input[pConsole->mode.cmucam.nInput++] = c;
      break;

    case aErrNotReady:
      readErr = aErrNotFound;
      break;
    
    case aErrIO:
      break;

    default:
      consoleErr = readErr;
      break;

    } /* switch */

  } while (readErr == aErrNone);

  return consoleErr;

} /* aConsoleMode_CMUCam_Slice */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleMode_CMUCam_Line
 *
 * Process a line of text that the user has input.  This line has
 * already been scanned for basic console commands.  In the CMUCam
 * mode, we just send the data to the camera and append the \r
 */

aErr aConsoleMode_CMUCam_Line(aConsole* pConsole,
			      const char* pLine)
{
  aErr consoleErr = aErrNone;

  /* check for commands */
  if (consoleErr == aErrNone) {
    aStream_Write(pConsole->ioLib, 
    		  pConsole->linkStreamRef,
    		  pLine, aStringLen(pLine),
    		  &consoleErr);
    if (consoleErr == aErrNone)
      aStream_Write(pConsole->ioLib, 
    		    pConsole->linkStreamRef,
    		    "\r", 1,
    		    &consoleErr);
  }

  return consoleErr;

} /* aConsoleMode_CMUCam_Line */


#endif /* aCMUCAM */
