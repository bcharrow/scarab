/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEAvmInternal.h					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Internal implementation of cross-platform TEA	   */
/*		virtual machine library.			   */
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

#ifndef _aTEAvmInternal_H_
#define _aTEAvmInternal_H_

#include "aErr.h"
#include "aIO.h"
#include "aTEA.h"
#include "aTEAvm.h"


#define fTEAvmInited	0x01


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * type definitions
 */

typedef struct aTEAProcess {
  unsigned char		stateReg;
  tADDRESS		cp;
  tADDRESS		codeSize;
  aTEAVMPopCmdProc	popCmdProc;
  void*			popCmdRef;
  aTEAVMFetchProc	fetchProc;
  void*			fetchRef;
  tADDRESS		sp;
  tSTACK		stackSize;
  char*			stack;
  aTEAProcessID		nPID;
  aTEAVMExitProc	exitProc;
  const void*		exitRef;
  aVMExit		eExitCode;
  unsigned long		wakeTick;
  struct aTEAProcess*	pNext;
  int			flags;
  struct aTEAvm*        pVM;
} aTEAProcess;

typedef struct aTEAvm {
  aIOLib		ioLib;
  int			flags;
  tSTACK		nMaxStackSize;
  unsigned int		nMaxProcesses;
  unsigned int		nNumProcesses;
  aTEAProcessID		nNextPID;
  aMemPoolRef		processPool;
  aTEAProcess*		pProcesses;
  aTEAProcess*		pCurrentProcess;
  char			fetchRegister[3];
  char			hexChar[16];
  unsigned char		opLengths[op_EXIT + 1];
  aTEAProcessID		nDeadPID; /* set when a process dies */
  aTEAVMIOPortProc	portCBProc;
  void*			portCBRef;
  int			check;
} aTEAvm;

#define aTEAVMCHECK	0xBBBB

#define aVALIDTEAVM(p)	if ((!p) || 				   \
			(((aTEAvm*)p)->check != aTEAVMCHECK))	   \
			  teavmErr = aErrParam;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * cross-platform routine definitions
 */

aErr aTEAvmInternal_Create(aTEAvm** ppVM);
aErr aTEAvmInternal_Initialize(aTEAvm* pVM,
			       unsigned int nMaxStackSize,
			       unsigned int nMaxProcesses,
			       aTEAVMIOPortProc portCBProc,
			       void* portCBRef);
aErr aTEAvmInternal_TimeSlice(aTEAvm* pVM,
			      aBool* pbProcessed);
aErr aTEAvmInternal_Launch(aTEAvm* pVM,
			   aTEAvmLaunchBlock* pLB);
aErr aTEAvmInternal_Step(aTEAvm* pVM, 
			 aTEAvmStepBlock* pSB);
aErr aTEAvmInternal_ViewStack(aTEAvm* pVM, 
			      const aTEAProcessID pid, 
			      const unsigned char nMode,
			      const tSTACK nStackOffset,
			      const unsigned char nBytes,
			      char* pData);
aErr aTEAvmInternal_Kill(aTEAvm* pVM, 
		         aTEAProcessID pid);
aErr aTEAvmInternal_Shutdown(aTEAvm* pVM);
aErr aTEAvmInternal_Destroy(aTEAvm* pVM);

#endif /* _aTEAvmInternal_H_ */
