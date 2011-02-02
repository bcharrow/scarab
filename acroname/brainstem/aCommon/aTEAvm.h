/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEAvm.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of cross-platform TEA virtual machine   */
/*		library.					   */
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

#ifndef _aTEAvm_H_
#define _aTEAvm_H_

#include "aIO.h"
#include "aTEA.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * opaque reference to the virtual machine library
 */

typedef aLIBREF aTEAvmLib;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * define symbol import mechanism
 */

#ifndef aTEAVM_EXPORT
#define aTEAVM_EXPORT aLIB_IMPORT
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * launch flags
 */

#define fTEAvmDebug		0x0001
#define fTEAvmSetPID		0x0002
#define fTEAvmHalted		0x0004
#define fTEAvmRobotAPI		0x0008
#define fTEAvmSymonym		0x0010


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * process state flags
 */

#define fPStateExited		0x01


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvm library manipulation routines
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

aTEAVM_EXPORT aLIBRETURN 
aTEAvm_GetLibRef(aTEAvmLib* pTEAvmRef, 
		 aErr* pErr);

aTEAVM_EXPORT aLIBRETURN 
aTEAvm_ReleaseLibRef(aTEAvmLib TEAvmRef, 
		     aErr* pErr);

aTEAVM_EXPORT aLIBRETURN 
aTEAvm_GetVersion(aTEAvmLib TEAvmRef,
		  unsigned long *pVersion,
		  aErr* pErr);

#ifdef __cplusplus 
}
#endif /* __cplusplus */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * VM fetch I/O callback
 */

typedef aErr (*aTEAVMFetchProc)(const aTEAProcessID nPID,
				const tADDRESS nOffset,
			     	char* pData,
			     	const tADDRESS nDataSize,
			     	void* ref);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * VM port I/O stack and control callback
 */

#define kTEAvmStackIOPush  1
#define kTEAvmStackIOPop  2
#define kTEAvmSetHalt  3
#define kTEAvmClearHalt  4

typedef void (*aTEAVMIOPortIOCallback)(aTEAvmLib vmRef,
		       		       aTEAProcessID pid,
		       		       char operation,
				       char* pData,
			     	       tBYTE dataSize);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * VM port I/O callback
 */

typedef aErr (*aTEAVMIOPortProc)(aTEAProcessID pid,
		       		 tADDRESS port,
			     	 aBool bRead,
			     	 char* data,
			     	 tBYTE dataSize,
				 aTEAVMIOPortIOCallback,
			     	 void* ref);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * VM pop command callback
 */

typedef aErr (*aTEAVMPopCmdProc)(aTEAProcessID pid,
			     	 char* data,
			     	 tBYTE dataSize,
			     	 void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * VM exit callback
 */

typedef aErr (*aTEAVMExitProc)(const aVMExit eExitCode,
		    	       const char* returnData,
		               const unsigned char returnDataSize,
			       const aTEAProcessID pid,
			       const void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * VM launch information block
 */

typedef struct aTEAvmLaunchBlock {
  aTEAVMPopCmdProc	popCmdProc;
  void*			popCmdRef;
  aTEAVMFetchProc	fetchProc;
  void*			fetchRef;
  tADDRESS		codeSize;
  char*			data;
  tSTACK		dataSize;
  tBYTE			retValSize;
  aTEAVMExitProc	exitProc;
  void*			exitRef;
  aTEAProcessID		pid;
  int			flags;
} aTEAvmLaunchBlock;

typedef struct aTEAvmStepBlock {
  aTEAProcessID		pid;
  char*			op;
  unsigned char 	processState;
  tADDRESS		pc;
  tSTACK 		sp;
  unsigned char 	stateReg;
} aTEAvmStepBlock;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

aTEAVM_EXPORT aLIBRETURN 
aTEAvm_Initialize(aTEAvmLib teavmRef,
		  unsigned int nMaxStackSize,
		  unsigned int nMaxProcesses,
		  aTEAVMIOPortProc portCBProc,
		  void* portCBRef,
		  aErr* pErr);

aTEAVM_EXPORT aLIBRETURN 
aTEAvm_TimeSlice(aTEAvmLib teavmRef,
		 aBool* pbProcessed,
		 aErr* pErr);

aTEAVM_EXPORT aLIBRETURN 
aTEAvm_Launch(aTEAvmLib teavmRef,
	      aTEAvmLaunchBlock* pLB,
	      aErr* pErr);

aTEAVM_EXPORT aLIBRETURN 
aTEAvm_Step(aTEAvmLib teavmRef,
	    aTEAvmStepBlock* pSB,
	    aErr* pErr);

aTEAVM_EXPORT aLIBRETURN 
aTEAvm_ViewStack(aTEAvmLib teavmRef,
		 const aTEAProcessID pid, 
		 const unsigned char nMode,
		 const tSTACK nStackOffset,
		 const unsigned char nBytes,
		 char* pData,
		 aErr* pErr);

aTEAVM_EXPORT aLIBRETURN 
aTEAvm_Kill(aTEAvmLib teavmRef, 
	    aTEAProcessID pid, 
	    aErr* pErr);

aTEAVM_EXPORT aLIBRETURN 
aTEAvm_Shutdown(aTEAvmLib teavmRef,
		aErr* pErr);

#ifdef __cplusplus 
}
#endif /* __cplusplus */

#endif /* _aTEAvm_H_ */
