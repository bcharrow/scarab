/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEADebugger.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent TEA Debugger  */
/*		server.						   */
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

#ifndef _aTEADebugger_H_
#define _aTEADebugger_H_

#ifdef aDEBUGGER

#include "aUI.h"
#include "aTEAvm.h"

#define aTEADGBMAXDISPLAYWIDTH		80


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

typedef struct aTEADebugger {

  void*				vpConsole;
  aIOLib			ioRef;
  aUILib			uiRef;
  aStreamRef			output;
  struct aTEADebugSession*	pSessions;
  unsigned char		        opLengths[op_EXIT + 1];
  
  /* rendering state for callbacks */
  int				nIndex;
  void*				vpRef;
  
} aTEADebugger;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr aTEADebugger_Create(aUILib uiRef,
			 void* vpConsole,
			 aStreamRef output,
			 aTEADebugger** ppDebugger);
aErr aTEADebugger_AddSession(aTEADebugger* pDebugger,
			     const char* filename,
			     const unsigned char module,
			     const aFileArea eSourceFileArea,
			     const aFileArea eObjectFileArea,
			     aTEAvmLaunchBlock* pLB);
aErr aTEADebugger_TimeSlice(aTEADebugger* pDebugger,
			    aBool* bChanged);
aErr aTEADebugger_Step(aTEADebugger* pDebugger,
		       unsigned int nSessionID);
aErr aTEADebugger_Kill(aTEADebugger* pDebugger,
		       unsigned int nSessionID);
aErr aTEADebugger_Destroy(aTEADebugger* pDebugger);

#else /* aDEBUGGER */

typedef void* aTEADebugger;

#endif /* aDEBUGGER */

#endif /* _aTEADebugger_H_ */

