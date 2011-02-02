/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsole.c                                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent BrainStem */
/*		console object.					   */
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

#include "aUtil.h"
#include "aConsole.h"
#include "aConsoleText.h"
#include "aConsole_Cmds.h"
#include "aConsole_Tests.h"
#include "aStream_TextLine.h"
#include "aStreamUtil.h"
#include "aCmd.tea"
#include "aAST_Text.h"
#include "aVersion.h"
#include "aConsoleTerminal.h"

#include "aConsoleMode_CMUCam.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sHBProc(
  const aBool bHBOn,
  void* vpConsole
);

static aErr sConsole_HTTPRequest(
  const char* pURL,
  aSymbolTableRef params,
  aStreamRef reply,
  void* vpRef
);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHBProc
 */

aErr sHBProc(
  const aBool bHBOn,
  void* vpConsole
)
{
  aConsole* pConsole = (aConsole*)vpConsole;

  aAssert(pConsole);
  aAssert(pConsole->check == aCONSOLECHECK);

  pConsole->bLEDOn = bHBOn;
  aConsole_UpdateHB(pConsole);

  return aErrNone;

} /* sHBProc */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aErr sConsole_HTTPRequest
 */

aErr sConsole_HTTPRequest(
  const char* pURL,
  aSymbolTableRef params,
  aStreamRef reply,
  void* vpRef
)
{
  aErr consoleErr = aErrNotFound;
  aConsole* pConsole = (aConsole*)vpRef;

  aAssert(pConsole);

  /* give the mode callback a crack at handling the url first */
  if (pConsole->modeHTTP)
    consoleErr = pConsole->modeHTTP(pURL, params, reply, vpRef);

  /* now see if it is a general console request */ 
  if (consoleErr == aErrNotFound) {
  }

  return consoleErr;

} /* aErr sConsole_HTTPRequest */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_Initialize
 */

aErr aConsole_Initialize(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;

  /* make sure we have a pointer to work with */
  if (pConsole == NULL)
    consoleErr = aErrParam;

  if (consoleErr == aErrNone) {
    /* first, blast any fields in the console struct and set 
     * the check byte.  We cache and restore the output dims
     * as platforms may need these preserved for initialization
     * output.
     */
    unsigned int nOutputLines = pConsole->nOutputLines;
    unsigned int nOutputWidth = pConsole->nOutputWidth;
    aBZero(pConsole, sizeof(aConsole));
    pConsole->check = aCONSOLECHECK;
    pConsole->nOutputLines = nOutputLines;
    pConsole->nOutputWidth = nOutputWidth;
    pConsole->rSP03.nVolume = 0;
    pConsole->rSP03.nPitch = 4;
    pConsole->rSP03.nSpeed = 2;
  }

  /* get the io lib ref */
  if (consoleErr == aErrNone)
    aIO_GetLibRef(&pConsole->ioLib, &consoleErr);

  /* get the ui lib ref */
  if (consoleErr == aErrNone)
    aUI_GetLibRef(&pConsole->uiLib, &consoleErr);

  /* get the stem lib ref */
  if (consoleErr == aErrNone)
    aStem_GetLibRef(&pConsole->stemLib, &consoleErr);

  /* get the vm lib ref */
  if (consoleErr == aErrNone)
    aTEAvm_GetLibRef(&pConsole->vmLib, &consoleErr);

  /* get the steep lib ref */
  if (consoleErr == aErrNone)
    aSteep_GetLibRef(&pConsole->steepLib, &consoleErr);

  /* get the leaf lib ref */
  if (consoleErr == aErrNone)
    aLeaf_GetLibRef(&pConsole->leafLib, &consoleErr);

  /* find the settings */
  if (consoleErr == aErrNone)
    aSettingFile_Create(pConsole->ioLib, MAXSETTING, SETTNGFILE,
    			&pConsole->settings, &consoleErr);

  /* set up the current mode */
  if (consoleErr == aErrNone) {
    char* pMode;
    aSettingFile_GetString(pConsole->ioLib, 
    			   pConsole->settings,
    			   MODESETTING, 
    			   &pMode,
    			   DEFAULTMODE,
    			   &consoleErr);
    if (consoleErr == aErrNone) {
    if (!aStringCompare(pMode, "SP03")) {
      pConsole->nMode = modeSP03;
#ifdef aTERMINAL
    } else if (!aStringCompare(pMode, "terminal")) {
      pConsole->nMode = modeTerminal;
#endif /* aTERMINAL */
#ifdef aCMUCAM
    } else if (!aStringCompare(pMode, "CMUCam")) {
      pConsole->nMode = modeCMUCam;
      pConsole->modeInit = aConsoleMode_CMUCam_Init;
      pConsole->modeCleanup = aConsoleMode_CMUCam_Cleanup;
      pConsole->modeDraw = aConsoleMode_CMUCam_Draw;
      pConsole->modeHTTP = aConsoleMode_CMUCam_HTTP;
      pConsole->modeSlice = aConsoleMode_CMUCam_Slice;
      pConsole->modeInput = aConsoleMode_CMUCam_Line;
#endif /* aCMUCAM */
    } else 
      pConsole->nMode = modeBrainStem;
    }
  }

  /* build the output */
  if (consoleErr == aErrNone) {
    int bufferSize;
    aSettingFile_GetInt(pConsole->ioLib, pConsole->settings, 
    			BUFFERKEY, &bufferSize,
    			DEFAULTBUFFER, &consoleErr);

    if (consoleErr == aErrNone)
      consoleErr = aTextDisplay_Create(kDisplayAll, 
    				       (unsigned int)bufferSize, 
    				       pConsole->nOutputWidth,
    				       &pConsole->outputRef);
    if (consoleErr == aErrNone) {
      pConsole->outputMask = kDisplayAll;
      aTextDisplay_AddLine(pConsole->outputRef, aT_WELCOME_TXT1, 
      			   kDisplayStatus);
      aTextDisplay_AddLine(pConsole->outputRef, aT_WELCOME_TXT2, 
      			   kDisplayStatus);
      consoleErr = aConsole_ShowVersion(pConsole, "", 
					aVERSION_PACK(aVERSION_MAJOR, 
					aVERSION_MINOR, 
					aCONSOLE_BUILD_NUM));
    }
  }

  /* build the http server */
  if (consoleErr == aErrNone)
    aHTTP_Create(pConsole->uiLib, 
    		 pConsole->settings, 
    		 sConsole_HTTPRequest,
    		 pConsole, 
    		 &pConsole->http,
    		 &consoleErr);

  /* see if we are licensed */
  if (consoleErr == aErrNone) {
    aErr cryptErr;
    aErr licenseErr = aErrNone;
    aStreamRef licenseFile;
    aStream_CreateFileInput(pConsole->ioLib, 
    			    aT_LICENSE_FILE, aFileAreaBinary, 
      			    &licenseFile, &cryptErr);
    if (cryptErr == aErrNone)
      licenseErr = aCrypt_Decode(licenseFile, 
      			         pConsole->userFName,
      			         pConsole->userLName,
      			         pConsole->userID,
      			         pConsole->vendorName);
    if ((cryptErr == aErrNone)
        && (licenseErr == aErrNone))
      pConsole->bLicensed = aTrue;

    if (cryptErr == aErrNone)
      aStream_Destroy(pConsole->ioLib, licenseFile, NULL);
  }

  if (consoleErr == aErrNone)
    aSettingFile_GetInt(pConsole->ioLib, pConsole->settings, 
    			DEBUGLEVELKEY, &pConsole->nDebugLevel,
    			DEFAULTDBGLEVEL, &consoleErr);
  
  if (consoleErr == aErrNone)
    aSettingFile_GetInt(pConsole->ioLib, pConsole->settings, 
    			WIDTHKEY, &pConsole->nWidth,
    			DEFAULTWIDTH, &consoleErr);

  if (consoleErr == aErrNone)
    aSettingFile_GetInt(pConsole->ioLib, pConsole->settings, 
    			HEIGHTKEY, &pConsole->nHeight,
    			DEFAULTHEIGHT, &consoleErr);

  if (consoleErr == aErrNone)
    aSettingFile_GetInt(pConsole->ioLib, pConsole->settings, 
    			LEFTKEY, &pConsole->nLeft,
    			DEFAULTLEFT, &consoleErr);

  if (consoleErr == aErrNone)
    aSettingFile_GetInt(pConsole->ioLib, pConsole->settings, 
    			TOPKEY, &pConsole->nTop,
    			DEFAULTTOP, &consoleErr);

  if (consoleErr == aErrNone)
    aSettingFile_GetULong(pConsole->ioLib, pConsole->settings, 
    			  CMDCOLORKEY, &pConsole->nCommandColor,
    			  DEFAULTCMDCOLOR, &consoleErr);

  if (consoleErr == aErrNone)
    aSettingFile_GetULong(pConsole->ioLib, pConsole->settings, 
    			  INCOLORKEY, &pConsole->nInputColor,
    			  DEFAULTINCOLOR, &consoleErr);

  if (consoleErr == aErrNone)
    aSettingFile_GetULong(pConsole->ioLib, pConsole->settings, 
    			  OUTCOLORKEY, &pConsole->nOutputColor,
    			  DEFAULTOUTCOLOR, &consoleErr);

  if (consoleErr == aErrNone)
    aSettingFile_GetULong(pConsole->ioLib, pConsole->settings, 
    			  STATCOLORKEY, &pConsole->nStatusColor,
    			  DEFAULTSTATCOLOR, &consoleErr);

  if (consoleErr == aErrNone)
    aSettingFile_GetULong(pConsole->ioLib, pConsole->settings, 
    			  MSGCOLORKEY, &pConsole->nMessageColor,
    			  DEFAULTMSGCOLOR, &consoleErr);

  if (consoleErr == aErrNone)
    aSettingFile_GetULong(pConsole->ioLib, pConsole->settings, 
    			  TIMEOUTKEY, &pConsole->nMSTimeout,
    			  DEFAULTTIMEOUT, &consoleErr);

  if (consoleErr == aErrNone)
    consoleErr = aStream_CreateConsoleOutput(
    			pConsole->ioLib, pConsole,
    			&pConsole->consoleOutputRef,
    			kDisplayStatus);

  if (consoleErr == aErrNone)
    consoleErr = aStream_CreateConsoleOutput(
    			pConsole->ioLib, pConsole,
    			&pConsole->consoleMessageRef,
    			kDisplayMessage);

  if (consoleErr == aErrNone)
    aStreamBuffer_Create(pConsole->ioLib,
    			 40,
    			 &pConsole->displayBuf,
    			 &consoleErr);

  if (consoleErr == aErrNone)
    aStreamBuffer_Create(pConsole->ioLib,
    			 40,
    			 &pConsole->lineBuffer,
    			 &consoleErr);

  if (consoleErr == aErrNone)
    aTEAvm_Initialize(pConsole->vmLib, 128, 4, 
    		      aConsole_VMPortCB, 
    		      pConsole,
    		      &consoleErr);

#ifdef aDEBUGGER
  if (consoleErr == aErrNone)
    consoleErr = aTEADebugger_Create(pConsole->uiLib,
    				     pConsole,
    				     pConsole->consoleOutputRef,
    				     &pConsole->pDebugger);

#endif /* aDEBUGGER */

  /* create the task list */
  if (consoleErr == aErrNone)
    consoleErr = aStemKernel_Create(pConsole->ioLib, 
    				    &pConsole->pKernel);

  /* display the licensing info */ 
  if (consoleErr == aErrNone) {
    char line[100];
    if (pConsole->bLicensed == aTrue) {
      aStringCopySafe(line, 100, aT_LICENSE_MSG);
      aStringCatSafe(line, 100, pConsole->userFName);
      aStringCatSafe(line, 100, " ");
      aStringCatSafe(line, 100, pConsole->userLName);
    } else {
      aStringCopySafe(line, 100, aT_EVALUATION_MSG);
    }
    consoleErr = aConsole_DisplayLine(pConsole, line, 
    				      kDisplayStatus);
  }

  /* display the mode we are in */
  if (consoleErr == aErrNone) {
    switch (pConsole->nMode) {

    case modeSP03:
      consoleErr = aConsole_DisplayLine(pConsole, 
      					aSTATUS_SP03_MODE, 
      					kDisplayStatus);
      break;

    case modeBrainStem:
      consoleErr = aConsole_DisplayLine(pConsole, 
      					aSTATUS_BRAINSTEM_MODE, 
      					kDisplayStatus);
      break;

#ifdef aTERMINAL
    case modeTerminal:
      consoleErr = aConsole_DisplayLine(pConsole, 
      					aSTATUS_TERMINAL_MODE, 
      					kDisplayStatus);
      break;
#endif /* aTERMINAL */ 
    } /* switch */
  }

  /* handle startup file */
  if (consoleErr == aErrNone)
    consoleErr = aConsole_HandleStartupFile(pConsole);

  aAssert(consoleErr == aErrNone);

  return(consoleErr);

} /* aConsole_Initialize routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_SetupLink
 */

aErr aConsole_SetupLink(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;
  aErr linkErr;

  switch (pConsole->nMode) {

  case modeBrainStem:
    aStreamUtil_CreateSettingStream(pConsole->ioLib,
    				    pConsole->uiLib,
    				    pConsole->stemLib,
    				    pConsole->settings,
    				    &pConsole->linkStreamRef,
    				    &linkErr);
    if (linkErr == aErrNone)
      consoleErr = aConsole_DisplayLine(pConsole, 
    				        aSTATUS_STREAM_SET,
    				        kDisplayStatus);
    break;

#ifdef aTERMINAL
  case modeTerminal:
#endif /* aTERMINAL */

#ifdef aCMUCAM
  case modeCMUCam:
#endif /* aCMUCAM */

  case modeSP03:
    aStreamUtil_CreateRawSettingStream(pConsole->ioLib,
    				       pConsole->uiLib,
    				       pConsole->settings,
    				       &pConsole->linkStreamRef,
    				       &linkErr);
    if (linkErr == aErrNone)
      consoleErr = aConsole_DisplayLine(pConsole, 
    				        aSTATUS_STREAM_SET,
    				        kDisplayStatus);
    break;

  } /* switch */

  /* install the HB proc */
  if (consoleErr == aErrNone)
    aStem_SetHBCallback(pConsole->stemLib, 
    		        sHBProc, 
    		        pConsole, 
    		        &consoleErr);

  /* initialize the mode */
  if ((consoleErr == aErrNone) && (pConsole->modeInit))
    consoleErr = pConsole->modeInit(pConsole);

  return consoleErr;

} /* aConsole_SetupLink */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_Shutdown
 */

aErr aConsole_Shutdown(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;

  aVALIDCONSOLE(pConsole);

  if (consoleErr == aErrNone) {
    if (pConsole->bShutdown) {
      aAssert(pConsole->bDone);
      return aErrNone;
    } else {
      pConsole->bShutdown = aTrue;
    }
    pConsole->bDone = aTrue;
  }

  if ((consoleErr == aErrNone) && pConsole->http) {
    aHTTP_Destroy(pConsole->uiLib, pConsole->http, &consoleErr);
    pConsole->http = NULL;
  }

  if ((consoleErr == aErrNone) && pConsole->modeCleanup)
    consoleErr = pConsole->modeCleanup(pConsole);

  if (consoleErr == aErrNone) {
    consoleErr = aStemKernel_Destroy(pConsole->pKernel);
    pConsole->pKernel = NULL;
  }

  aAssert(consoleErr == aErrNone);

#ifdef aTESTS
  if (consoleErr == aErrNone)
    consoleErr = aConsole_DestroyTests(pConsole);
#endif /* aTESTS */

  aAssert(consoleErr == aErrNone);

#ifdef aDEBUGGER
  if ((consoleErr == aErrNone) &&
      (pConsole->pDebugger != NULL)) {
    aTEADebugger_Destroy(pConsole->pDebugger);
    pConsole->pDebugger = NULL;
  }
#endif /* aDEBUGGER */

  aAssert(consoleErr == aErrNone);

  if (consoleErr == aErrNone)
    aTEAvm_Shutdown(pConsole->vmLib, &consoleErr);

  aAssert(consoleErr == aErrNone);

  if (consoleErr == aErrNone) {
    aStream_Destroy(pConsole->ioLib, 
    		    pConsole->consoleOutputRef, &consoleErr);
    pConsole->consoleOutputRef = NULL;
  }

  if (consoleErr == aErrNone) {
    aStream_Destroy(pConsole->ioLib, 
    		    pConsole->consoleMessageRef, &consoleErr);
    pConsole->consoleMessageRef = NULL;
  }

  if (consoleErr == aErrNone) {
    aStream_Destroy(pConsole->ioLib, 
    		    pConsole->displayBuf, &consoleErr);
    pConsole->displayBuf = NULL;
  }

  if (consoleErr == aErrNone) {
    aStream_Destroy(pConsole->ioLib, 
    		    pConsole->lineBuffer, &consoleErr);
    pConsole->lineBuffer = NULL;
  }

  /* release the leaf library reference */
  if ((consoleErr == aErrNone) &&
      !aLeaf_ReleaseLibRef(pConsole->leafLib, &consoleErr))
    pConsole->leafLib = NULL;

  aAssert(consoleErr == aErrNone);

  /* release the steep library reference */
  if ((consoleErr == aErrNone) &&
      !aSteep_ReleaseLibRef(pConsole->steepLib, &consoleErr))
    pConsole->steepLib = NULL;

  aAssert(consoleErr == aErrNone);

  /* release the vm library reference */
  if ((consoleErr == aErrNone) &&
      !aTEAvm_ReleaseLibRef(pConsole->vmLib, &consoleErr))
    pConsole->vmLib = NULL;

  aAssert(consoleErr == aErrNone);

  /* release the stem library reference */
  if ((consoleErr == aErrNone) &&
      !aStem_ReleaseLibRef(pConsole->stemLib, &consoleErr))
    pConsole->stemLib = NULL;

  aAssert(consoleErr == aErrNone);

  /* clean up the output */
  if (consoleErr == aErrNone) {
    aTextDisplay_Destroy(pConsole->outputRef);
    pConsole->outputRef = NULL;
  }

  aAssert(consoleErr == aErrNone);

  /* toss the settings file */
  if ((consoleErr == aErrNone) && 
      (pConsole->settings != NULL)) {
    aSettingFile_Destroy(pConsole->ioLib, pConsole->settings, 
    			 &consoleErr);
    pConsole->settings = NULL;
  }

  aAssert(consoleErr == aErrNone);

  /* release the ui library reference */
  if ((consoleErr == aErrNone) &&
      !aUI_ReleaseLibRef(pConsole->uiLib, &consoleErr))
    pConsole->uiLib = NULL;

  aAssert(consoleErr == aErrNone);

  /* release the linkStream if the mode didn't use the stem
   * library */
  if (consoleErr == aErrNone) {
    switch (pConsole->nMode) {

#ifdef aTERMINAL
    case modeTerminal:
#endif /* aTERMINAL */
#ifdef aCMUCAM
    case modeCMUCam:
#endif /* aCMUCAM */

    case modeSP03:
      if (pConsole->linkStreamRef) {
        aStream_Destroy(pConsole->ioLib,
                        pConsole->linkStreamRef,
                        &consoleErr);
        pConsole->linkStreamRef = NULL;
      }
      break;
    } /* switch */
  }

  aAssert(consoleErr == aErrNone);

  /* release the io library reference */
  if ((consoleErr == aErrNone) &&
      !aIO_ReleaseLibRef(pConsole->ioLib, &consoleErr))
    pConsole->ioLib = NULL;

  aAssert(consoleErr == aErrNone);

  return(consoleErr);

} /* aConsole_Shutdown routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_TimeSlice
 *
 * Handles cross-platform packet handling and processing tasks.
 *
 * Returns true if tasks were completed, and false if there were 
 * none to complete.
 */

aBool aConsole_TimeSlice(aConsole* pConsole, aErr* pErr)
{
  aBool bSliced = aFalse;
  aErr consoleErr = aErrNone;

  if (pConsole->bDone) {
    *pErr = aErrNone;
    return aFalse;
  }

  aVALIDCONSOLE(pConsole);

  /* first, retreive any link packets that can be found */
  if ((consoleErr == aErrNone)
      && (pConsole->linkStreamRef != NULL) 
      && (pConsole->bDone != aTrue)) {
    
    switch (pConsole->nMode) {

    case modeBrainStem:
      {
        aErr packetErr;
        aPacketRef packetRef;

        aStem_GetPacket(pConsole->stemLib, 
        		NULL, NULL, 0L, &packetRef, &packetErr);
        if (packetErr == aErrNone) {
          consoleErr = aStemKernel_AddPacket(pConsole->pKernel, 
      					     packetRef);
	}
        else if ((packetErr == aErrIO)
    	         && pConsole->relayStreamRef) {
          aStem_SetStream(pConsole->stemLib,
      		          NULL, kStemRelayStream, &consoleErr);
          pConsole->relayStreamRef = NULL;
          aConsole_DisplayLine(pConsole, "relay dropped", 
      			       kDisplayStatus);
        }
        else if ((packetErr != aErrNotFound) 
		 && (packetErr != aErrTimeout)) {
	  char buf[100];
	  aSNPRINTF(buf, 100, "error = %d", packetErr);
	  aConsole_DisplayLine(pConsole, buf, kDisplayStatus);
          consoleErr = packetErr;
	}
      }
      break;

#ifdef aTERMINAL
    case modeTerminal:
      consoleErr = aConsoleTerminal_TimeSlice(pConsole);
      break;
#endif /* aTERMINAL */

    default:
      if (pConsole->modeSlice)
        consoleErr = pConsole->modeSlice(pConsole);
      break;
    } /* nMode switch */
  } /* if */

  /* then, let the vm get some cycles for virtual 
   * machine processes */
  if (consoleErr == aErrNone)
    aTEAvm_TimeSlice(pConsole->vmLib, &bSliced, &consoleErr);

  /* check for HTTP requests */
  if ((consoleErr == aErrNone) && (pConsole->http))
    aHTTP_TimeSlice(pConsole->uiLib,
    		    pConsole->http,
    		    &bSliced,
    		    &consoleErr);

#ifdef aDEBUGGER
  if ((consoleErr == aErrNone)
      && (pConsole->pDebugger))
    consoleErr = aTEADebugger_TimeSlice(pConsole->pDebugger,
    					&bSliced);
#endif /* aDEBUGGER */

  /* process any packets that have come in */
  if (consoleErr == aErrNone) {
    aPacketRef packet;
    while(aStemKernel_PacketAvailable(pConsole->pKernel, &packet)
          && (consoleErr == aErrNone))
      consoleErr = aConsole_HandlePacket(pConsole, packet);
    if (consoleErr)
      printf("ERROR !!!!!!! handle packet");
  }

#ifdef aTESTS
  /* let the tests continue if needed */
  if (consoleErr == aErrNone)
    consoleErr = aConsole_HandleTests(pConsole);
#endif /* aTESTS */

  if (pErr != NULL)
    *pErr = consoleErr;

  return bSliced;

} /* aConsole_TimeSlice */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleLine
 *
 * Handles interactive input lines.
 */

aErr aConsole_HandleLine(aConsole* pConsole,
			 const char *line,
			 const aBool bDisplay)
{
  aErr consoleErr = aErrNone;
  aStreamRef dataStream;
  char dbgBuff[MAXINPUTLINE + 9];

  aVALIDCONSOLE(pConsole);

  /* dump the line to the debug file */
  if (consoleErr == aErrNone) {
    aStringCopySafe(dbgBuff, MAXINPUTLINE + 9, "console: ");
    aStringCatSafe(dbgBuff, MAXINPUTLINE + 9, line);
    aStem_DebugLine(pConsole->stemLib, dbgBuff, &consoleErr);
  }

  /* display the command */
  if (consoleErr == aErrNone)
    consoleErr = aConsole_DisplayLine(pConsole, line, 
    				      kDisplayCommand);

  /* convert the line to a data stream */
  if ((consoleErr == aErrNone) && (*line != '\0')) {
    switch (pConsole->nMode) {

    case modeBrainStem:
    case modeSP03:
      consoleErr = aStream_Create_TextLine_Input(pConsole->ioLib, 
    					         line, &dataStream);

      /* let the console handle the command as a stream */
      if (consoleErr == aErrNone)
        consoleErr = aConsole_CommandStream(pConsole, 
    					    dataStream,
    					    bDisplay);
      break;

#ifdef aTERMINAL
    case modeTerminal:
      consoleErr = aConsoleTerminal_HandleLine(pConsole,
      					       line);
      break;
#endif /* aTERMINAL */

    default:
      if (pConsole->modeInput)
        consoleErr = pConsole->modeInput(pConsole, line);
      break;

    } /* switch */
  }

  return(consoleErr);

} /* aConsole_HandleLine routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandlePacket
 *
 * Handles a packet.
 */

aErr aConsole_HandlePacket(aConsole* pConsole, 
			   aPacketRef inPacketRef)
{
  aErr consoleErr = aErrNone;
  unsigned char address;
  unsigned char len;
  char data[aSTEMMAXPACKETBYTES];
  char d1;
  unsigned char d2;
  aTEAProcessID pid;
  tBYTE byteVal;
  tSHORT shortVal;
  aBool bTest = aFalse;

  aVALIDCONSOLE(pConsole);
 
  if (consoleErr == aErrNone)
    aPacket_GetData(pConsole->stemLib, inPacketRef, 
  		    &address, &len, data, &consoleErr);
  
  if (consoleErr == aErrNone) {

    if (len == 0) 
      goto display_packet;

    d1 = data[0];
    d2 = (unsigned char)data[0];

    switch(d2) {

    /* messages */
    case cmdMSG:

      /* switch on message type */
      switch (data[1]) {
  
      /* vmExit looks like:
       * byte 0 - cmdMSG
       * byte 1 - vmExit
       * byte 2 - pid
       * byte 3 - exitCode
       * byte 4 - data (optional length-4 bytes)
       */
      case vmExit:
        aAssert(len >= 4);
        pid = (aTEAProcessID)data[2];

#ifdef aTESTS
        {
          char* dp = (len > 4) ? &data[4] : NULL;
          bTest = aConsole_CheckTestExit(pConsole, address,
            		             pid, (aVMExit)data[3], dp,
              		             (unsigned char)(len - 4));
        }
#endif /* aTESTS */
        
        /* build up and display the status string */
        if (!bTest && (pConsole->nDebugLevel > 0)) {
          char num[10];

          aConsole_BufferOutput(pConsole, aVM_PROCESS_EXIT);
          aConsole_BufferOutput(pConsole, ": ");
          if (address == 0)
            aConsole_BufferOutput(pConsole, "host");
          else {
            aStringFromInt(num, address);
            aConsole_BufferOutput(pConsole, num);
          }
          aConsole_BufferOutput(pConsole, ",");

          aStringFromInt(num, pid);
          aConsole_BufferOutput(pConsole, num);

          aConsole_BufferOutput(pConsole, ",");
 
          aStringFromInt(num, data[3]);
          aConsole_BufferOutput(pConsole, num);

          aConsole_BufferOutput(pConsole, ",");
	  if (len - 4 == 0) {
	    aStringCopySafe(num, 10, aAST_VOID);
	  } else if (len - 4 == 1) {
	    byteVal = data[4];
 	    aStringFromInt(num, byteVal);
	  } else {
	    shortVal = aUtil_RetrieveShort(&data[4]);
 	    aStringFromInt(num, shortVal);
	  }
          aConsole_BufferOutput(pConsole, num);
          consoleErr = aConsole_DisplayBuffer(pConsole, 
          				      kDisplayStatus);
        }
        break;

      default:
        {
          char line[40];
          aStemMsg_Format((unsigned char)data[1], 
          		  address, line, 40);
          if (consoleErr == aErrNone)
            consoleErr = aConsole_DisplayLine(pConsole, line, 
          				      kDisplayMessage);
        }
        break;
      } /* switch */
      break; /* cmdMSG */

    case cmdVM_MSG:
#ifdef aTESTS
      bTest = aConsole_CheckTestDisplay(pConsole, address,
      					(aTEAProcessID)data[1],
      					data[2]);
#endif /* aTESTS */
      if (!bTest)
        consoleErr = aConsole_VMDisplay(pConsole, address, 
        			        (aTEAProcessID)data[1], 
        			        &data[2]);
      break;

display_packet:
    default:
      { 
        char line[40];
        if (consoleErr == aErrNone)
          aPacket_Format(pConsole->stemLib, inPacketRef, 
		       line, 40, &consoleErr);
         if (consoleErr == aErrNone)
           consoleErr = aConsole_DisplayLine(pConsole, line, 
        				  kDisplayInput);
      }
      break;

    } /* switch */
  }

  if (consoleErr == aErrNone)
    aPacket_Destroy(pConsole->stemLib, inPacketRef, &consoleErr);

  return consoleErr;

} /* end of aConsole_HandlePacket routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_ParseCommand
 */

aErr aConsole_ParseCommand(aConsole* pConsole,
			   aToken* pCommandToken,
			   aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;

  aAssert(pCommandToken->eType == tkIdentifier);

  if (!aStringCompare(pCommandToken->v.identifier, "exit")) {
    aConsole_HandleExit(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "batch")) {
    aConsole_HandleBatch(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "launch")) {
    aConsole_HandleLaunch(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "dsm")) {
    aConsole_HandleDsm(pConsole, pCommandToken, tokenizer);
#ifdef aTESTS
  } else if (!aStringCompare(pCommandToken->v.identifier, "test")) {
    aConsole_HandleTest(pConsole, pCommandToken, tokenizer);
#endif /* aTESTS */
  } else if (!aStringCompare(pCommandToken->v.identifier, "load")) {
    aConsole_HandleLoad(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "unload")) {
    aConsole_HandleUnLoad(pConsole, pCommandToken, tokenizer);
#ifdef aDEBUGGER
  } else if (!aStringCompare(pCommandToken->v.identifier, "debug")) {
    aConsole_HandleDebug(pConsole, pCommandToken, tokenizer);
#if 0
  } else if (!aStringCompare(pCommandToken->v.identifier, "serve")) {
    aConsole_HandleServe(pConsole, pCommandToken, tokenizer);
#endif
#endif /* aDEBUGGER */
  } else if (!aStringCompare(pCommandToken->v.identifier, "pp")) {
    aConsole_HandleSteep(pConsole, fSteepPreprocess, 
    			 pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "ast")) {
    aConsole_HandleSteep(pConsole, fSteepSAST, 
    			 pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "SP03")) {
    aConsole_HandleSP03(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "stat")) {
    aConsole_HandleStat(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "steep")) {
    aConsole_HandleSteep(pConsole, fSteepGenerateCode, 
    			 pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "last")) {
    aConsole_HandleLeaf(pConsole, fLeafAST, 
    			pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "leaf")) {
    aConsole_HandleLeaf(pConsole, fLeafGenerateCode, 
    			pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "relay")) {
    aConsole_HandleRelay(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "raw")) {
    aConsole_HandleRaw(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "say")) {
    aConsole_HandleSay(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "dump")) {
    aConsole_HandleDump(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "help")) {
    aConsole_HandleHelp(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "makedump")) {
    aConsole_HandleMakeDump(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, "braindump")) {
    aConsole_HandleBrainDump(pConsole, pCommandToken, tokenizer);
  } else {
    aConsole_DisplayLine(pConsole, aCMD_ILLEGAL_CMD, kDisplayStatus);
  }

  return consoleErr;

} /* end of aConsole_ParseCommand */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_ParsePacket
 */

aErr aConsole_ParsePacket(aConsole* pConsole,
			  aToken* pFirstToken,
			  aTokenizerRef tokenizer,
			  aBool bDisplay)
{
  aErr consoleErr = aErrNone;
  unsigned char address;
  unsigned char length = 0;
  char data[aSTEMMAXPACKETBYTES];

  aAssert(pConsole);
  aAssert(pFirstToken);
  aAssert(pFirstToken->eType == tkInt);

  /* we don't parse packets in other modes */
  if ((consoleErr == aErrNone)
      && (pConsole->nMode != modeBrainStem)) {
    aConsole_DisplayLine(pConsole, 
    			 aCMD_INVALID_MODE, 
    			 kDisplayStatus);
    consoleErr = aErrParse;
  }

  if ((pFirstToken->v.integer > 254) ||
      ((pFirstToken->v.integer & 0x01) &&
       (pFirstToken->v.integer != aSTEM_MAGICADDR))) {
    aConsole_DisplayLine(pConsole, aPKT_ILLEGAL_ADDR, 
    			 kDisplayStatus);
    consoleErr = aErrParse;
  }

  if (consoleErr == aErrNone) {
    address = (unsigned char)pFirstToken->v.integer;
    consoleErr = aConsole_ParseRaw(pConsole, 
    				   tokenizer,
    				   data, 
    				   &length, 
    				   aSTEMMAXPACKETBYTES);
    if (consoleErr == aErrParse)
      aConsole_DisplayLine(pConsole, aPKT_ILLEGAL_LEN, 
      			   kDisplayStatus);
  }

  if (consoleErr == aErrNone) {
    aPacketRef packet;
    char line[40];
    aPacket_Create(pConsole->stemLib,
    		   address,
    		   length,
    		   data,
    		   &packet,
    		   &consoleErr);

    if (consoleErr == aErrNone)
      aStem_SendPacket(pConsole->stemLib,
      		       packet, &consoleErr);

    if (bDisplay == aTrue) {
      if (consoleErr == aErrNone)
        aPacket_Format(pConsole->stemLib,
    		       packet, line,
    		       40,
    		       &consoleErr);
      if (consoleErr == aErrNone)
        aConsole_DisplayLine(pConsole, line, kDisplayOutput);
    }
  }
  
  return consoleErr;

} /* aConsole_ParsePacket */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_ParseRaw
 */

aErr aConsole_ParseRaw(aConsole* pConsole,
		       aTokenizerRef tokenizer,
		       char* data,
		       unsigned char* pDataLen,
		       unsigned char maxLen)
{
  aErr consoleErr = aErrNone;
  unsigned char length = 0;
  aToken* pToken;
  aBool bDone = aFalse;

  while ((consoleErr == aErrNone) 
         && (bDone == aFalse)
  	 && !aTokenizer_Next(pConsole->ioLib, tokenizer, 
  	 		     &pToken, NULL)) {

    switch (pToken->eType) {

    /* ; can end a line for raw data input */    
    case tkSpecial:
      if (pToken->v.special == ';')
        bDone = aTrue;
      else
        consoleErr  = aErrParse;
      break;

    case tkString: 
      {
        char* p = pToken->v.string;
        while(*p && (consoleErr == aErrNone)) {
	  if (length >= maxLen) {
            consoleErr = aErrParse;
	  } else 
	    data[length++] = *p++;
        } /* while */
      }
      break;

    case tkInt:
      if (length >= maxLen) {
        consoleErr = aErrParse;
      } else
        data[length++] = (char)pToken->v.integer;
      break;
      
    default:
      consoleErr = aErrParse;
      break;

    } /* tk.eType switch */

    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);

  } /* while */
  
  if ((consoleErr == aErrNone)
      && (pDataLen != NULL))
    *pDataLen = length;
 
  return consoleErr;

} /* aConsole_ParseRaw */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_CommandStream
 */

aErr aConsole_CommandStream(aConsole* pConsole,
			    aStreamRef stream,
			    const aBool bDisplay)
{
  aErr consoleErr = aErrNone;
  aTokenizerRef tokenizer;
  aToken* pToken;
  aBool bAvail = aFalse;

  aVALIDCONSOLE(pConsole);

  if (consoleErr == aErrNone)
    aTokenizer_Create(pConsole->ioLib, 
    		      stream, 
    		      aT_CMDLINENAME,
    		      aFileAreaUser,
    		      NULL,
    		      NULL, 
    		      &tokenizer, 
    		      &consoleErr);

  /* inspect the first token to see if it is a command */
  if (consoleErr == aErrNone)
    bAvail = !aTokenizer_Next(pConsole->ioLib, tokenizer, 
    			      &pToken, NULL);

  if (bAvail == aTrue) {
    switch (pToken->eType) {

    case tkInt:
      consoleErr = aConsole_ParsePacket(pConsole,
      					pToken,
      					tokenizer,
      					bDisplay);
      if (consoleErr == aErrParse)
        consoleErr = aErrNone;
      break;

    case tkIdentifier:
      consoleErr = aConsole_ParseCommand(pConsole,
      					 pToken,
      					 tokenizer);
      break;

    default:
      aConsole_DisplayLine(pConsole, aCMD_ILLEGAL_INPUT, 
      			   kDisplayStatus);
      break;

    } /* switch */

    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, 
    		       &consoleErr);
  }

  if (consoleErr == aErrNone)
    aTokenizer_Destroy(pConsole->ioLib, tokenizer, &consoleErr);

  return(consoleErr);

} /* aConsole_CommandStream routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_Compile
 */

aErr aConsole_Compile(aConsole* pConsole,
		      const int compileFlags,
		      aStreamRef out,
		      aStreamRef err,
		      const char* inputName,
		      const aFileArea eInputArea,
		      const char* outputName,
		      const aFileArea eOutputArea)
{
  aErr consoleErr = aErrNone;
  aStreamRef sourceStream = NULL;
  aStreamRef destStream = NULL;

  /* open the input file */
  if (consoleErr == aErrNone)    
    aStream_CreateFileInput(pConsole->ioLib,
    			    inputName,
    			    eInputArea,
    			    &sourceStream,
    			    &consoleErr);

  /* open the output file */
  if (consoleErr == aErrNone)    
    aStream_CreateFileOutput(pConsole->ioLib,
    			     outputName,
    			     eOutputArea,
    			     &destStream,
    			     &consoleErr);    			     

  /* compile the source code */
  if (consoleErr == aErrNone) {
    aSteep_Compile(pConsole->steepLib,
    		   sourceStream,
    		   inputName,
    		   compileFlags,
    		   destStream,
    		   out,
    		   err,
    		   &consoleErr);
    
    if (consoleErr == aErrParse)
      consoleErr = aErrNone;
    aAssert(consoleErr == aErrNone);
  }

  /* close the streams regardless of outcome */
  if (sourceStream != NULL)
    aStream_Destroy(pConsole->ioLib, sourceStream, NULL);
  if (destStream != NULL)
    aStream_Destroy(pConsole->ioLib, destStream, NULL);

  return consoleErr;

} /* aConsole_Compile */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_Leaf
 */

aErr aConsole_Leaf(aConsole* pConsole,
		   const int compileFlags,
		   aStreamRef out,
		   aStreamRef err,
		   const char* inputName,
		   const aFileArea eInputArea,
		   const char* outputName,
		   const aFileArea eOutputArea)
{
  aErr consoleErr = aErrNone;
  aStreamRef sourceStream = NULL;
  aStreamRef destStream = NULL;

  /* open the input file */
  if (consoleErr == aErrNone)    
    aStream_CreateFileInput(pConsole->ioLib,
    			    inputName,
    			    eInputArea,
    			    &sourceStream,
    			    &consoleErr);

  /* open the output file */
  if (consoleErr == aErrNone)
    aStream_CreateFileOutput(pConsole->ioLib,
    			     outputName,
    			     eOutputArea,
    			     &destStream,
    			     &consoleErr);

  /* compile the source code */
  if (consoleErr == aErrNone) {
    aLeaf_Compile(pConsole->leafLib,
    		  sourceStream,
    		  inputName,
    		  compileFlags,
    		  destStream,
    		  out,
    		  err,
    		  &consoleErr);
    
    if (consoleErr == aErrParse)
      consoleErr = aErrNone;
    aAssert(consoleErr == aErrNone);
  }

  /* close the streams regardless of outcome */
  if (sourceStream != NULL)
    aStream_Destroy(pConsole->ioLib, sourceStream, NULL);
  if (destStream != NULL)
    aStream_Destroy(pConsole->ioLib, destStream, NULL);

  return consoleErr;

} /* aConsole_Leaf */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_Launch
 */

aErr aConsole_Launch(
  aConsole* pConsole,
  const char* fileName,
  const aFileArea eArea,
  const char* data,
  const unsigned char dataLen,
  aTEAVMExitProc exitProc,
  const void* exitRef,
  int flags,
  aTEAProcessID* pPID)
{
  aErr consoleErr = aErrNone;
  aFileRef codeFile = NULL;
  aTEAvmLaunchBlock lb;
  aConsoleVMCode* pCode = NULL;

  aVALIDCONSOLE(pConsole);

  /* initialize the launch block */
  if (consoleErr == aErrNone)
    aBZero(&lb, sizeof(aTEAvmLaunchBlock));

  /* here we are good to go, read in the code */
  if ((consoleErr == aErrNone)
       && (aFile_Open(pConsole->ioLib, 
    		      fileName, 
    		      aFileModeReadOnly,
    		      eArea,
    		      &codeFile, 
    		      &consoleErr))) {
    switch(consoleErr) {
    case aErrNotFound:
      aConsole_DisplayLine(pConsole, aCMD_FILE_NOT_FOUND, 
      			   kDisplayStatus);
      break;
    default:
      aConsole_DisplayLine(pConsole, aCMD_FILE_ERROR, 
      			   kDisplayStatus);
      break;
    } /* switch */
  }

  /* get the file size */
  if (consoleErr == aErrNone) {
    unsigned long size;
    aFile_GetSize(pConsole->ioLib, 
    		  codeFile, 
    		  &size, 
    		  &consoleErr);
    if (consoleErr == aErrNone)
      lb.codeSize = (tADDRESS)size;
  }

  /* build a code structure for the running program */
  if (consoleErr == aErrNone){
    pCode = (aConsoleVMCode*)aMemAlloc(sizeof(aConsoleVMCode));
    if (!pCode)
      consoleErr = aErrMemory;
    else {
      aBZero(pCode, sizeof(aConsoleVMCode));
      pCode->nSize = lb.codeSize;
      pCode->pConsole = pConsole;
    }
  }

  /* allocate the code storage */
  if (consoleErr == aErrNone) {
    pCode->pStore = (char*)aMemAlloc((aMemSize)lb.codeSize);
    if (pCode->pStore == NULL)
      consoleErr = aErrMemory;
  }

  /* read in the code */
  if (consoleErr == aErrNone) {
    aFile_Read(pConsole->ioLib, 
    	       codeFile, 
    	       pCode->pStore, 
    	       lb.codeSize, 
    	       NULL, 
    	       &consoleErr);
  }

  /* launch the process */
  if (consoleErr == aErrNone) {
    lb.fetchProc = aConsole_VMFetch;
    lb.fetchRef = (void*)pCode;
    lb.data = (char*)data;
    lb.dataSize = dataLen;
    lb.exitProc = exitProc;
    lb.exitRef = (void*)exitRef;
    lb.flags = flags;
    if (flags & fTEAvmSetPID)
      lb.pid = *pPID;

    /* check for successful launch */   
    if (!aTEAvm_Launch(pConsole->vmLib, &lb, &consoleErr)) {

      /* set the PID we were assigned if not prescribed */
      if (!(flags & fTEAvmSetPID))
        *pPID = lb.pid;   

      /* link in the pCode block for cleanup on exit */
      pCode->nPID = lb.pid;
      pCode->pNext = pConsole->pVMCode;
      pConsole->pVMCode = pCode;
    }
  }

  /* close the code file regardless of outcome */
  if (codeFile != NULL)
    aFile_Close(pConsole->ioLib, codeFile, NULL);

  return consoleErr;

} /* aConsole_Launch */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_VMFetch
 */

aErr aConsole_VMFetch(
  const aTEAProcessID nPID,
  const tADDRESS nOffset,
  char* pData,
  const tADDRESS nDataSize,
  void* ref
)
{
  aErr consoleErr = aErrNone;
  aConsoleVMCode* pCode = (aConsoleVMCode*)ref;

  /* should error check */
#if 0
  if (nPID != pCode->nPID)
    consoleErr = aErrUnknown;
  else
#endif
  if (pCode->nSize < nOffset + nDataSize)
    consoleErr = aErrUnknown;
  else
    aMemCopy(pData, &pCode->pStore[nOffset], nDataSize);

  return consoleErr;
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_VMExit
 */

aErr aConsole_VMExit(
  const aVMExit eExitCode,
  const char* returnData,
  const unsigned char returnDataSize,
  const aTEAProcessID pid,
  const void* ref
)
{
  aErr consoleErr = aErrNone;
  aConsole* pConsole = (aConsole*)ref; 
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
  unsigned char returnCt = returnDataSize;

  aAssert(pConsole);

  data[0] = (char)cmdMSG;
  data[1] = vmExit;
  data[2] = (char)pid;
  data[3] = eExitCode;

  /* go through the list of code and remove this process ID's 
   * code storage */  
  if (consoleErr == aErrNone) {
    aConsoleVMCode* pPrev = NULL; 
    aConsoleVMCode* pTemp = pConsole->pVMCode;
    while (pTemp && (pTemp->nPID != pid)) {
      pPrev = pTemp;
      pTemp = pTemp->pNext;
    }
    aAssert(pTemp);
    if (!pPrev) {
      pConsole->pVMCode = pTemp->pNext;
    } else {
      pPrev->pNext = pTemp->pNext;
    }
    aMemFree(pTemp->pStore);
    aMemFree(pTemp);
  }
  
  if (eExitCode == aVMExitNormal) {
    if (returnDataSize > 0)
      aMemCopy(&data[4], returnData, returnCt);
  } else {
    /* process is in undetermined state after error */
    /* process must return 0 bytes */
    returnCt = 0;
  }
  
  aPacket_Create(pConsole->stemLib, 0, 
  		 (unsigned char)(4 + returnCt), 
  		 data, &packet, &consoleErr);

  if (consoleErr == aErrNone)
    aStemKernel_AddPacket(pConsole->pKernel, packet);

  return consoleErr;

} /* aConsole_VMExit */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_Load
 */

aErr aConsole_Load(aConsole* pConsole,
		   aStreamRef fileStream,
		   unsigned char module,
		   unsigned char slot)

{
  aErr consoleErr = aErrNone;
  aStreamRef slotStream = NULL;

  aVALIDCONSOLE(pConsole);

  /* we don't parse packets in other modes */
  if ((consoleErr == aErrNone)
      && (pConsole->nMode != modeBrainStem)) {
    aConsole_DisplayLine(pConsole, 
    			 aCMD_INVALID_MODE, 
    			 kDisplayStatus);
    consoleErr = aErrParse;
  }

  if (consoleErr == aErrNone) {
    static int a = 0;
    char num[10];
    char line[100];
    aStringFromInt(num, a);
    a++;
    aStringCopySafe(line, 100, "load number ");
    aStringCatSafe(line, 100, num);
    aStem_DebugLine(pConsole->stemLib, line, NULL);
  }

  /* try to build a connection to the slot on the module */
  if (consoleErr == aErrNone) {
    aStem_CreateTEAFileOutput(pConsole->stemLib,
    			      module,
    			      slot,
    			      &slotStream, 
    			      &consoleErr);
    if (consoleErr != aErrNone)
      aConsole_DisplayLine(pConsole, 
      			   aCMD_SLOTINIT_FAILED,
      			   kDisplayStatus);
  }

  /* do the actual loading */
  if (consoleErr == aErrNone) {
    aErr copyErr;
    aStream_Flush(pConsole->ioLib,
    			fileStream,
    			slotStream,
    			&copyErr);
    /* now determine whether we succeeded or not */
    if (copyErr == aErrNone) {
      if (pConsole->nDebugLevel > 0)
        consoleErr = aConsole_DisplayLine(pConsole, 
        				  aCMD_LOAD_SUCCEEDED,
        				  kDisplayStatus);
    } else {
      aConsole_DisplayLine(pConsole, 
      			   aCMD_LOAD_FAILED,
      			   kDisplayStatus);
      consoleErr = aErrIO;
    }
  }

  /* clean up the slot stream either way if it was opened */
  if (slotStream != NULL) {
    aStream_Destroy(aStreamLibRef(slotStream), slotStream, NULL);
    slotStream = NULL;
  }
  
  return consoleErr;

} /* aConsole_Load */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_VMDisplay
 */

aErr aConsole_VMDisplay(aConsole* pConsole, 
			unsigned char module,
			aTEAProcessID pid,
			char* displayData)
{
  aErr consoleErr = aErrNone;

  aVALIDCONSOLE(pConsole);

  if (consoleErr == aErrNone) {  
    aStream_Write(pConsole->ioLib, 
    		  pConsole->consoleMessageRef,
    		  displayData, 1, &consoleErr);
  }

  return consoleErr;

} /* aConsole_VMDisplay */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_Batch
 */

aErr aConsole_Batch(aConsole* pConsole,
		    const char* batchfile,
		    const aFileArea eArea,
		    const aBool bDisplay)
{
  aErr consoleErr = aErrNone;
  aStreamRef batchStream = NULL;

  aVALIDCONSOLE(pConsole);
  
  if (consoleErr == aErrNone) {
    /* here we are good to go, start reading the batchfile */
    if (aStream_CreateFileInput(pConsole->ioLib, 
    				batchfile, 
    				eArea,
    			    	&batchStream, 
    			    	&consoleErr)) {
      switch(consoleErr) {
      case aErrNotFound:
        aConsole_DisplayLine(pConsole, aCMD_FILE_NOT_FOUND, 
        		     kDisplayStatus);
        break;
      default:
        aConsole_DisplayLine(pConsole, aCMD_FILE_ERROR, 
        		     kDisplayStatus);
        break;
      }
    }
  }

  /* here we have an open file for reading */
  if (consoleErr == aErrNone) {
    aErr streamErr = aErrNone;
    char inputline[MAXINPUTLINE];

    if ((bDisplay == aTrue) 
        && (pConsole->nDebugLevel > 0)) {
      aConsole_DisplayLine(pConsole, aCMD_BATCH_START, 
      			   kDisplayStatus);
      aConsole_BufferOutput(pConsole, " \"");
      aConsole_BufferOutput(pConsole, batchfile);
      aConsole_BufferOutput(pConsole, "\"");
      aConsole_DisplayBuffer(pConsole, kDisplayStatus);
    }
 
    while ((streamErr == aErrNone) &&
           (consoleErr == aErrNone)) {
      if (!aStream_ReadLine(pConsole->ioLib, batchStream,
      		            inputline, MAXINPUTLINE,
      		            &streamErr)) {
        /* MRW needed for long Garcia batch files */
        /* otherwise the Stems would overflow */
        aIO_MSSleep(pConsole->ioLib, 10, NULL);
        consoleErr = aConsole_HandleLine(pConsole, 
        				 inputline, 
        				 bDisplay);
      }
    }

    if ((bDisplay == aTrue) 
        && (pConsole->nDebugLevel > 0)) {
      aConsole_DisplayLine(pConsole, aCMD_BATCH_END, 
      			   kDisplayStatus);
    }

  }

  /* close the batch file regardless of outcome */
  if (batchStream != NULL)
    aStream_Destroy(pConsole->ioLib, batchStream, NULL);

  return consoleErr;
  
} /* aConsole_Batch */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_ShowVersion
 *
 * version organized as:
 * top 4 bits = major
 * next 4 bits = minor
 * bottom 3 bytes = build
 */

aErr aConsole_ShowVersion(aConsole* pConsole,
			  const char* entity, 
			  const unsigned long version)
{
  char line[100];
  char num[10];
  int major = aVERSION_UNPACK_MAJOR(version);
  int minor = aVERSION_UNPACK_MINOR(version);
  int build = aVERSION_UNPACK_BUILD(version);

  aStringCopySafe(line, 100, entity);
  aStringCatSafe(line, 100, " version: ");
  aStringFromInt(num, major);
  aStringCatSafe(line, 100, num);
  aStringCatSafe(line, 100, ".");
  aStringFromInt(num, minor);
  aStringCatSafe(line, 100, num);
  aStringCatSafe(line, 100, ", build ");
  aStringFromInt(num, build);
  aStringCatSafe(line, 100, num);

  return aConsole_DisplayLine(pConsole, line, kDisplayStatus);

} /* aConsole_ShowVersion */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleStartupFile
 */

aErr aConsole_HandleStartupFile(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;
  
  if (consoleErr == aErrNone) {
    aErr ioErr;
    aFileRef testFile;

    aAssert(pConsole->ioLib);

    /* check for debug file */
    aFile_Open(pConsole->ioLib, STARTUPFILE, 
  	       aFileModeReadOnly, aFileAreaBinary,
  	       &testFile, &ioErr);

    /* just close it if you found it */
    if (ioErr == aErrNone)
      aFile_Close(pConsole->ioLib, testFile, &ioErr);

    /* then handle it like a batch file */
    if (ioErr == aErrNone)
      consoleErr = aConsole_Batch(pConsole, STARTUPFILE, 
      				  aFileAreaBinary, aTrue);
  }
  
  
  return consoleErr;

} /* aConsole_HandleStartupFile */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_BufferOutput
 */

aErr aConsole_BufferOutput(aConsole* pConsole,
			   const char* text)
{
  aErr consoleErr = aErrNone;
  aIOLib ioRef;
  
  aVALIDCONSOLE(pConsole);
  
  if (consoleErr == aErrNone)
    ioRef = pConsole->ioLib;
  
  if (consoleErr == aErrNone)
    aStream_Write(ioRef, pConsole->displayBuf,
                  text, aStringLen(text), &consoleErr);
 
  return consoleErr;

} /* aConsole_BufferOutput */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_DisplayBuffer
 */

aErr aConsole_DisplayBuffer(aConsole* pConsole, 
			    const aTextDisplayType type)
{
  aErr consoleErr = aErrNone;
  aIOLib ioRef;
  
  aVALIDCONSOLE(pConsole);
  
  if (consoleErr == aErrNone)
    ioRef = pConsole->ioLib; 

  /* add in the null terminator */
  if (consoleErr == aErrNone)
    aStream_Write(ioRef, pConsole->displayBuf,
                  "\0", 1, &consoleErr);
  
  /* dump to the display */
  if (consoleErr == aErrNone) {
    char *pLine;
    if (!aStreamBuffer_Get(ioRef, pConsole->displayBuf,
    		           NULL, &pLine, &consoleErr))
      aConsole_DisplayLine(pConsole, pLine, type);
  }
 
  /* reset the buffer */
  if (consoleErr == aErrNone)
    aStream_Flush(ioRef, pConsole->displayBuf, 
    		        NULL, &consoleErr);

  return consoleErr;

} /* aConsole_DisplayBuffer */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_UpdateDrawing
 */

aErr aConsole_UpdateDrawing(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;
  
  aVALIDCONSOLE(pConsole);
  
  aAssert(pConsole->drawGD);

  if ((consoleErr == aErrNone) && pConsole->modeDraw)
    consoleErr = pConsole->modeDraw(pConsole);    

  return consoleErr;

} /* aConsole_UpdateDrawing */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_DrawingDoubleClick
 */

aErr aConsole_DrawingDoubleClick(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;
  unsigned long inetAddr;
  
  aVALIDCONSOLE(pConsole);
  
  /* this isn't set up right now because some machines 
   * firewall HTTP traffic on the external address */
  /* find the machines IP address */
  if (consoleErr == aErrNone)
    aIO_GetInetAddr(pConsole->ioLib,
    		    &inetAddr,
    		    &consoleErr);

  /* launch the browser window */
  if ((consoleErr == aErrNone) && (pConsole->http)) {
    char addr[16];
    char url[100];
    aUtil_FormatInetAddr(addr, inetAddr, NULL);
    aStringCopySafe(url, 100, "http://");
    aStringCatSafe(url, 100, addr);
    aStringCatSafe(url, 100, ":8000/draw");
    aBrowser_LaunchURL(pConsole->uiLib, url, &consoleErr);    
  }

  return consoleErr;

} /* aConsole_DrawingDoubleClick */
