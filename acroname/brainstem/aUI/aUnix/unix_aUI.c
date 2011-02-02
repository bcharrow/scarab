/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aUI.c                                                */
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

#if defined(aUNIX)

#include <stdlib.h>

#include "unix_aUI.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUI_GetLibRef
 */

aLIBRETURN 
aUI_GetLibRef(aUILib* pRef,
	      aErr* pErr)
{
  aErr uiErr = aErrNone;
  aUnixUI* pUI = NULL;
  
  if (pRef == NULL)
    uiErr = aErrParam;

  if (uiErr == aErrNone) {
    pUI = (aUnixUI*)aMemAlloc(sizeof(aUnixUI));
    if (pUI == NULL)
      uiErr = aErrMemory;
  }

  if (uiErr == aErrNone) {
    aBZero(pUI, sizeof(aUnixUI));
    uiErr = aUIInternal_Initialize((aUI*)pUI);
  }

#ifndef aNOX
  /* Show that we have not initialized X windows stuff yet. We do this
   * here instead of just initializing so that it only uses X if it
   * needs to (in case it isn't available
   */
  if (uiErr == aErrNone) {
    pUI->bXInited = 0;
  }
#endif /* aNOX */

  if (uiErr == aErrNone)
    *pRef = pUI;

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* end of aUI_GetLibRef routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUI_ReleaseLibRef
 */

aLIBRETURN 
aUI_ReleaseLibRef(aUILib uiRef,
		  aErr* pErr)
{
  aErr uiErr = aErrNone;
  aUnixUI* pUI = (aUnixUI*)uiRef;

  aVALIDUI(uiRef);

  if (uiErr == aErrNone)
    uiErr = aUIInternal_Cleanup((aUI*)pUI);

#ifndef aNOX
  if ((uiErr == aErrNone) && pUI->pPointBuffer) {
    aMemFree(pUI->pPointBuffer);
    pUI->pPointBuffer = NULL;
  }
#endif /* aNOX */

  if (uiErr == aErrNone)
    aMemFree(pUI);

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* end of aUI_ReleaseLibRef routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aDialog_Message
 */

aLIBRETURN 
aDialog_Message(aUILib uiRef,
		const char* message,
		aUIDialogIdleProc idleProc,
		void* idleRef,
		aErr* pErr)
{
  aErr uiErr = aErrNone;
  //  aUnixUI* pUI = (aUnixUI*)uiRef;

  aVALIDUI(uiRef);
  if ((uiErr == aErrNone) && (message == NULL))
    uiErr = aErrParam;
  
  if (uiErr == aErrNone) {
  }

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aDialog_Message */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aDialog_PickFile
 */

aLIBRETURN 
aDialog_PickFile(aUILib uiRef,
		 const char* pMessage,
		 char* pFileName,
		 const aFileArea eFileArea,
		 aUIPickFileFilterProc filterProc,
		 aUIDialogIdleProc idleProc,
		 void* idleRef,
		 aErr* pErr)
{
  aErr uiErr = aErrNone;
  //  aUnixUI* pUI = (aUnixUI*)uiRef;

  aVALIDUI(uiRef);
  if ((uiErr == aErrNone) && (pMessage == NULL))
    uiErr = aErrParam;
  
  if (uiErr == aErrNone) {
  }

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aDialog_PickFile */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBrowser_LaunchURL
 */

aLIBRETURN 
aBrowser_LaunchURL(aUILib uiRef,
		   const char* pURL,
		   aErr* pErr)
{
  aErr uiErr = aErrNone;

  aVALIDUI(uiRef);

#if 0
  if (uiErr == aErrNone) {
    char line[100];
    aStringCopy(line, "mozilla ");
    aStringCat(line, pURL);
    if (system(line))
      uiErr = aErrIO;
  }
#endif

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aBrowser_LaunchURL */


#endif /* aUNIX */

