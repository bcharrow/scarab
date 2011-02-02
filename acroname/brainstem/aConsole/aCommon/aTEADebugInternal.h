/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEADebugInternal.h					   */
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

#ifndef _aTEADebugInternal_H_
#define _aTEADebugInternal_H_

#ifdef aDEBUGGER

#include "aTEADebugSession.h"

aErr aTEADebugInternal_Launch(aTEADebugSession* pSession,
			      aTEAvmLaunchBlock* pLB);
aErr aTEADebugInternal_Step(aTEADebugSession* pSession);
aErr aTEADebugInternal_Kill(aTEADebugSession* pSession);
aErr aTEADebugInternal_ViewStack(aTEADebugSession* pSession,
			         const unsigned char nMode,
			         const tSTACK nStackOffset,
			         const unsigned char nBytes,
			         char* pData);

#endif /* aDEBUGGER */

#endif /* _aTEADebugInternal_H_ */
