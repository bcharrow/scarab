/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEADebugInternal.c					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Routines the debugger uses to interact with TEAvm  */
/*              processes running within the Console.		   */
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

#include "aConsole.h"
#include "aTEADebugInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugInternal_Launch
 */

aErr aTEADebugInternal_Launch(aTEADebugSession* pSession,
			      aTEAvmLaunchBlock* pLB)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole;

  aAssert(pSession);
  pConsole = (aConsole*)(pSession->pDebugger->vpConsole);
  aAssert(pConsole);
   
  aTEAvm_Launch(pConsole->vmLib, pLB, &dsErr);
  
  return dsErr;

} /* aTEADebugInternal_Launch */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugInternal_Step
 */

aErr aTEADebugInternal_Step(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)(pSession->pDebugger->vpConsole);

  aAssert(pSession);
  aAssert(pConsole);  

  aTEAvm_Step(pConsole->vmLib, &(pSession->SB), &dsErr);

  return dsErr;

} /* aTEADebugInternal_Step */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugInternal_ViewStack
 */

aErr aTEADebugInternal_ViewStack(aTEADebugSession* pSession,
				 const unsigned char nMode,
				 const tSTACK nStackOffset,
				 const unsigned char nBytes,
				 char* pData)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)(pSession->pDebugger->vpConsole);

  aAssert(pSession);
  aAssert(pConsole);  

  aTEAvm_ViewStack(pConsole->vmLib, 
  		   (unsigned char)sProcessID(pSession),
    		   nMode, nStackOffset, nBytes, pData, &dsErr);
    
  return dsErr;

} /* aTEADebugInternal_ViewStack */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugInternal_Kill
 */

aErr aTEADebugInternal_Kill(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)(pSession->pDebugger->vpConsole);

  aAssert(pSession);
  aAssert(pConsole);  

  aTEAvm_Kill(pConsole->vmLib, (unsigned char)sProcessID(pSession), &dsErr);

  return dsErr;

} /* aTEADebugInternal_Kill */

#endif /* aDEBUGGER */
