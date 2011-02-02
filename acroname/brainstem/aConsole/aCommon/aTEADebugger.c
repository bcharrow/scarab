/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEADebugger.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent TEA       */
/*		Debugger server.				   */
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

#ifdef aDEBUGGER

#include "aUtil.h"
#include "aTEADebugger.h"
#include "aTEADebugSession.h"
#include "aConsole.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

#if 0
static aErr sTEADebugger_Request(const char* pURL,
				 aSymbolTableRef params,
				 aStreamRef reply,
				 void* vpRef);
#endif

static aErr sTEADebugger_FindSession(aTEADebugger* pDebugger,
				     unsigned int nSessionID,
				     aTEADebugSession** ppSession);



#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEADebugger_Request
 */

aErr sTEADebugger_Request(const char* pURL,
			  aSymbolTableRef params,
			  aStreamRef reply,
			  void* vpRef)
{
  aErr dbgErr = aErrNone;
  aTEADebugger* pDebugger = (aTEADebugger*)vpRef;
  aTEADebugSession* pSession = NULL;

  /* here, we find the session based on the id parameter the
   * browser sent us */
  if ((dbgErr == aErrNone) && params) {
    int i;
    dbgErr = aSymbolTable_GetInt(pDebugger->ioRef, params, "id", &i);
    if (dbgErr == aErrNone) {
      dbgErr = sTEADebugger_FindSession(pDebugger, 
      					(unsigned int)i, 
      					&pSession);
    } else if ((dbgErr == aErrNotFound)
               && !aStringCompare(pURL, "/debugger"))
    {
      /* session with invalid ID */
      /* debugger with no session ID */
      pSession = NULL;
      dbgErr = aErrNone;
    
    }      					
  }
  
  /* handle any additional parameters for any request */
  if ((dbgErr == aErrNone) && params)
    dbgErr = aTEADebugSession_ProcessParams(pSession, params);

  if (dbgErr == aErrNone) {
    if (!aStringCompare(pURL, "/c")) {
      dbgErr = aTEADebugSession_RenderCode(pSession, reply);
    } else if (aStringCompare(pURL, "/k") == 0) {
      dbgErr = aTEADebugSession_RenderStack(pSession, reply);
    } else if (aStringCompare(pURL, "/v") == 0) {
      dbgErr = aTEADebugSession_RenderVariables(pSession, reply);
    } else if (aStringCompare(pURL, "/s") == 0) {
      dbgErr = aTEADebugSession_RenderCallStack(pSession, reply);
    } else if (aStringCompare(pURL, "/r") == 0) {
      dbgErr = aTEADebugSession_RenderRegisters(pSession, reply);
    } else if (aStringCompare(pURL, "/b") == 0) {
      dbgErr = aTEADebugSession_RenderButtons(pSession, reply);
    } else if (aStringCompare(pURL, "/l") == 0) {
      dbgErr = aTEADebugSession_RenderLogo(pSession, reply);
    } else if (aStringCompare(pURL, "/debugger") == 0) {
      dbgErr = aTEADebugSession_RenderDebugger(pDebugger, pSession, reply);
    } else {
      dbgErr = aErrNotFound;
    }
  }

  return dbgErr;

} /* sTEADebugger_Request */
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEADebugger_FindSession
 */

aErr sTEADebugger_FindSession(aTEADebugger* pDebugger,
			      unsigned int nSessionID,
			      aTEADebugSession** ppSession)
{
  aErr dbgErr = aErrNotFound;  
  aTEADebugSession* pTemp = pDebugger->pSessions;

  while (pTemp && (pTemp->nSessionPID != nSessionID)) {
    pTemp = pTemp->pNext;
  }

  if (pTemp) {
    *ppSession = pTemp;
    dbgErr = aErrNone;
  }
  
  return dbgErr;
  
} /* sTEADebugger_FindSession */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugger_Create
 */

aErr aTEADebugger_Create(aUILib uiRef,
			 void* vpConsole,
			 aStreamRef output,
			 aTEADebugger** ppDebugger)
{
  aErr dbgErr = aErrNone;
  aTEADebugger* pDebugger = NULL;
  aIOLib ioRef = NULL;
  
  /* check for bad parameters */
  if (!ppDebugger)
    dbgErr = aErrParam;
  if (dbgErr == aErrNone) {
    ioRef = aStreamLibRef(output);
    if (!ioRef)
      dbgErr = aErrParam;
  }

  /* allocate the debugger object */  
  if (dbgErr == aErrNone) {
    pDebugger = (aTEADebugger*)aMemAlloc(sizeof(aTEADebugger));
    if (pDebugger) {
      aBZero(pDebugger, sizeof(aTEADebugger));
      pDebugger->ioRef = ioRef;
      pDebugger->uiRef = uiRef;
      pDebugger->vpConsole = vpConsole;
      pDebugger->output = output;
      aTEA_SetOpCodeLengths(pDebugger->opLengths);
    } else
      dbgErr = aErrMemory;
  }

  if (dbgErr == aErrNone) {
    *ppDebugger = pDebugger;
  } else {
    aTEADebugger_Destroy(pDebugger);
  }

  return aErrNone;

} /* aTEADebugger_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugger_TimeSlice
 */

aErr aTEADebugger_TimeSlice(aTEADebugger* pDebugger,
			    aBool* bChanged)
{
  aErr dbgErr = aErrNone;
  struct aTEADebugSession* pSession;
  struct aTEADebugSession* pNext;
  struct aTEADebugSession* pPrev;

  if (!pDebugger)
    dbgErr = aErrParam;

  if (dbgErr == aErrNone) {
    pSession = pDebugger->pSessions;
    pPrev = NULL;    		    
    while (pSession && (dbgErr == aErrNone)) {
      if (pSession->bKillNow) {

        /* unlink dead process */
        pNext = pSession->pNext;
        if (pPrev == NULL) {
          pDebugger->pSessions = pNext;
        } else {
          pPrev->pNext = pNext;
        }
        /* destroy process */
        dbgErr = aTEADebugSession_Destroy(pSession);
  
      } else {
      
        /* normal operation */
        if (pSession->bRunning)
          aTEADebugSession_TimeSlice(pSession);
        pNext = pSession->pNext;
        pPrev = pSession;

      }
      pSession = pNext;
    }
  }
  
  return dbgErr;

} /* aTEADebugger_TimeSlice */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugger_AddSession
 */

aErr aTEADebugger_AddSession(aTEADebugger* pDebugger,
			     const char* pFilename,
			     const unsigned char module,
			     const aFileArea eSourceFileArea,
			     const aFileArea eObjectFileArea,
			     aTEAvmLaunchBlock* pLB)
{
  aErr dbgErr = aErrNone;
  aTEADebugSession* pSession = NULL;
  aTEADbgCreateBlock cb;

  
  cb.pDebugger = pDebugger;
  cb.pFilename = pFilename;
  cb.eSourceFileArea = eSourceFileArea;
  cb.eObjectFileArea = eObjectFileArea;
  cb.module = module;
  cb.pLB = pLB;
  dbgErr = aTEADebugSession_Create(&cb, &pSession);
  
  /* link it into the session list */
  if (dbgErr == aErrNone) {
    pSession->pNext = pDebugger->pSessions;
    pDebugger->pSessions = pSession;
  }

  /* clean up if something bad happened */  
  if (dbgErr != aErrNone) {
    if (pSession)
      aTEADebugSession_Destroy(pSession);
  }

  return dbgErr;

} /* aTEADebugger_AddSession */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugger_Step
 *
 * Used primarily by the tests to walk through programs via the
 * debugger.
 */

aErr aTEADebugger_Step(aTEADebugger* pDebugger,
		       unsigned int nSessionID)
{
  aErr dbgErr = aErrNone;
  aTEADebugSession* pSession = NULL;

  if (dbgErr == aErrNone)
    dbgErr = sTEADebugger_FindSession(pDebugger, nSessionID, 
    				      &pSession);
  
  if (dbgErr == aErrNone)
    dbgErr = aTEADebugSession_StepIn(pSession);

  return dbgErr;

} /* aTEADebugger_Step */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugger_Kill
 *
 * Used primarily by the tests to walk through programs via the
 * debugger.
 */

aErr aTEADebugger_Kill(aTEADebugger* pDebugger,
		       unsigned int nSessionID)
{
  aErr dbgErr = aErrNone;
  aTEADebugSession* pPrev = NULL;
  aTEADebugSession* pTemp = pDebugger->pSessions;


  /* search the list of sessions, remove, and destroy by id */ 
  while (pTemp) {
    if (pTemp->nSessionPID == nSessionID) {

      /* unlink it from the list */
      if (!pPrev)
        pDebugger->pSessions = pTemp->pNext;
      else {
        pPrev->pNext = pTemp->pNext;
      }

      /* kill it */
      aTEADebugSession_Destroy(pTemp);
      
      /* we are done */
      break;

    } else {
      pPrev = pTemp;
      pTemp = pTemp->pNext;
    }
  }

  /* if we didn't find it, let the caller know */  
  if (!pTemp)
    dbgErr = aErrNotFound;

  return dbgErr;

} /* aTEADebugger_Kill */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugger_Destroy
 */

aErr aTEADebugger_Destroy(aTEADebugger* pDebugger)
{
  aErr dbgErr = aErrNone;

  if (pDebugger) {
  
    while (pDebugger->pSessions) {
      aTEADebugSession* pTemp = pDebugger->pSessions;
      pDebugger->pSessions = pTemp->pNext;
      aTEADebugSession_Destroy(pTemp);
    }

    aMemFree(pDebugger);
  }

  return dbgErr;

} /* aTEADebugger_Destroy */

#endif /* aDEBUGGER */

