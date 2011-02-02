/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEADebugSession.h					   */
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

#ifndef _aTEADebugSession_H_
#define _aTEADebugSession_H_

#ifdef aDEBUGGER

#include "aTEAvm.h"
#include "aTEADebugger.h"
#include "aTextDisplay.h"

#define aDBG_STACK_SIZE	128

#define aDBG_BTN_GO		0
#define aDBG_BTN_STEPIN		1
#define aDBG_BTN_STEPOUT	2
#define aDBG_BTN_STEPOVER	3
#define aDBG_BTN_STOP		4
#define aDBG_BTN_KILL		5

#define aDBG_MASK_BREAKPT	0x01
#define aDBG_MASK_SRCLINE	0x02
#define aDBG_MASK_FLAG2		0x04
#define aDBG_MASK_FLAG3		0x08
#define aDBG_MASK_FLAG4		0x10
#define aDBG_MASK_FLAG5		0x20
#define aDBG_MASK_FLAG6		0x40
#define aDBG_MASK_FLAG7		0x80

#define aDBG_DIRTY_LOGO		0x01
#define aDBG_DIRTY_CODE		0x02
#define aDBG_DIRTY_STACK	0x04
#define aDBG_DIRTY_CALLS	0x08
#define aDBG_DIRTY_REGS		0x10
#define aDBG_DIRTY_VARS		0x20
#define aDBG_DIRTY_BUTTONS	0x40
#define	aDBG_DIRTY_ALL		0x7F


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

typedef struct aTEADebugInstr {
  char		op[3];
  char		cFlags;
  int		nAddress;
} aTEADebugInstr;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

typedef struct aTEADebugStackFrame {
  tADDRESS			address;
  const char*			pName;
  struct aTEADebugStackFrame*	pNext;
} aTEADebugStackFrame;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * routine types used for abstracting where the TEAvm is running
 */
typedef aErr (*aTEAvmLaunchProc)(struct aTEADebugSession* pSession,
				 aTEAvmLaunchBlock* pLB);
typedef aErr (*aTEAvmStepProc)(struct aTEADebugSession* pSession);
typedef aErr (*aTEAvmKillProc)(struct aTEADebugSession* pSession);
typedef aBool (*aTEAvmStepEndCheck)(struct aTEADebugSession* pSession);
typedef aErr (*aTEAvmViewStackProc)(struct aTEADebugSession* pSession,
				    const unsigned char nMode,
				    const tSTACK nStackOffset,
				    const unsigned char nBytes,
				    char* pData);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

typedef struct aTEADebugSession {

  /* basic process info */
  tADDRESS			codeSize;
  char*				code;
  tBYTE				retValSize;

  /* polymorphic goodies */
  aTEAvmStepBlock		SB;
  aTEAvmStepProc		step;
  aTEAvmKillProc		kill;
  aTEAvmLaunchProc		launch;
  aTEAvmViewStackProc		viewstack;
  aTEAvmStepEndCheck		endcheck;

  /* file and display */
  struct aTEADebugger*		pDebugger;
  struct aTEADebugSession* 	pNext;
  unsigned int			nSessionPID;
  char				pSourceName[aFILE_NAMEMAXCHARS];
  aFileArea			eSourceArea;
  aFileArea			eObjectArea;
  aTextDisplayRef		sourceDisplayRef;
  unsigned int			nSourceLines;
  unsigned int			nCurrentLine;
  unsigned int			nWindowLines;
  unsigned int			nCodeRenderStart;
  unsigned int			nCodeRenderEnd;

  /* renderer data state */
  int				nDirtyFrames;
  int				nInstrCt;
  int				nInstrIndex;
  int				nStepOverTarg;
  char				cStack[aDBG_STACK_SIZE];
  aBool				fStackChg[aDBG_STACK_SIZE];
  aBool				bRunning;
  aBool				bStopNow;
  aBool				bPrepareToKill;
  aBool				bKillNow;
  aBool				bRefreshing;
  struct aTEADebugInstr*	pInstr;
  aTEADebugStackFrame*		pStackFrames;
  int				nRegInfo[6];
  aStreamRef			lineBuffer;

  /* identical to console debugger */
  tADDRESS			cp;
  tADDRESS			sp;
  unsigned char			module; /* 0 for host */
  unsigned char			statusReg;
  unsigned char			processState;

  /* rendering state for callbacks */
  int				nIndex;
  void*				vpRef;
  aTextDisplayRef		output;
} aTEADebugSession;




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Creation and Destruction
 *
 * The contstructor should be minimal and only allocate things when
 * they are needed.  The destructor should completely clean up the
 * Session and all it's associated VM processes, structures etc. 
 *
 * The creation block allows us to change the effective creation
 * parameters without re-visiting the general API on all the 
 * various platforms.
 */

typedef struct aTEADbgCreateBlock {
  aTEADebugger*			pDebugger;
  const char* 			pFilename;
  aFileArea			eSourceFileArea;
  aFileArea			eObjectFileArea;
  unsigned char			module;
  aTEAvmLaunchBlock*		pLB;
} aTEADbgCreateBlock;

aErr aTEADebugSession_Create(const aTEADbgCreateBlock* pCB,
			     aTEADebugSession** ppSession);

aErr aTEADebugSession_Destroy(aTEADebugSession* pSession);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Session Actions
 *
 * These functions should be the actual stage changing functions.
 * They should resolve to the appropriate calls into the VM via
 * packets or direct calls to a VM running in the aTEAvm library.
 */


aErr aTEADebugSession_Go(aTEADebugSession* pSession);
aErr aTEADebugSession_Stop(aTEADebugSession* pSession);
aErr aTEADebugSession_StepIn(aTEADebugSession* pSession);
aErr aTEADebugSession_StepOut(aTEADebugSession* pSession);
aErr aTEADebugSession_StepOver(aTEADebugSession* pSession);
aErr aTEADebugSession_Kill(aTEADebugSession* pSession);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Parameter callbacks
 *
 * This should check for all relevant operations and change the 
 * state of the session accordingly.
 */

aErr aTEADebugSession_ProcessParams(aTEADebugSession* pSession,
			            aSymbolTableRef params);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Rendering callbacks
 *
 * These should reflect the current session state.
 */

aErr aTEADebugSession_TimeSlice(aTEADebugSession* pSession);

aErr aTEADebugSession_RenderControls(aTEADebugSession* pSession, 
			             aStreamRef browser);

aErr aTEADebugSession_RenderCallStack(aTEADebugSession* pSession, 
			              aStreamRef browser);

aErr aTEADebugSession_RenderVariables(aTEADebugSession* pSession, 
			              aStreamRef browser);

aErr aTEADebugSession_RenderStack(aTEADebugSession* pSession, 
			          aStreamRef browser);

aErr aTEADebugSession_RenderRegisters(aTEADebugSession* pSession, 
			          aStreamRef browser);

aErr aTEADebugSession_RenderCode(aTEADebugSession* pSession, 
			         aStreamRef browser);

aErr aTEADebugSession_RenderButtons(aTEADebugSession* pSession, 
			            aStreamRef browser);

aErr aTEADebugSession_RenderLogo(aTEADebugSession* pSession, 
			         aStreamRef browser);

aErr aTEADebugSession_RenderDebugger(aTEADebugger* pDebugger, 
				     aTEADebugSession* pSession, 
			             aStreamRef browser);

char sProcessID(aTEADebugSession* pSession);


#endif /* aDEBUGGER */

#endif /* _aTEADebugSession_H_ */

