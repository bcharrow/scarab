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

#include "aStem.h"
#include "aUtil.h"
#include "aIOPorts.tea"
#include "aTEAvmInternal.h"
#include "aOSDefs.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * TEA file signature and header size
 */

#define aTEAHEADERSIZE 6


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local routines
 */

static aErr sTEAvm_FreeProcess(aTEAvm* pVM, aTEAProcess* pProcess);
static aTEAProcess* sTEAvm_ProcessFromID(aTEAvm* pVM,
					 aTEAProcessID pid);
static aErr sTEAvm_Fetch(aTEAvm* pVM);
static void sTEAvm_CallbackStackManipulate(aTEAvmLib vmRef,
					  aTEAProcessID pid,
					  char operation,
					  char* data,
					  tBYTE dataSize);
static aErr sTEAvm_Execute(aTEAvm* pVM,
			   aTEAProcess* pProcess);
static aErr sTEAvm_Abort(aTEAvm* pVM, 
			 aTEAProcess* pProcess,
			 aVMExit exitCode);
static aErr sTEAvm_StackPushByte(aTEAvm* pVM,
				 aTEAProcess* pProcess,
				 unsigned char val);
static aErr sTEAvm_StackPushShort(aTEAvm* pVM,
				  aTEAProcess* pProcess,
				  unsigned short val);
static aErr sTEAvm_StackPushAddress(aTEAvm* pVM,
				    aTEAProcess* pProcess,
				    tADDRESS val);
static aErr sTEAvm_StackPopByte(aTEAvm* pVM,
				aTEAProcess* pProcess,
				unsigned char* pVal);
static aErr sTEAvm_StackPopShort(aTEAvm* pVM,
				 aTEAProcess* pProcess,
				 unsigned short* pVal);
static aErr sTEAvm_StackPopAddress(aTEAvm* pVM,
				   aTEAProcess* pProcess,
				   tADDRESS* pVal);
static aErr sTEAvm_StackPopOffset(aTEAvm* pVM,
				  aTEAProcess* pProcess,
				  tOFFSET* pVal);
static aErr sTEAvm_StackGetByte(aTEAvm* pVM,
			 	aTEAProcess* pProcess,
			 	tSTACK offset,
			 	unsigned char* pVal);
static aErr sTEAvm_StackGetByteAbs(aTEAvm* pVM,
			 	   aTEAProcess* pProcess,
			 	   tSTACK offset,
			 	   unsigned char* pVal);
static aErr sTEAvm_StackPutByte(aTEAvm* pVM,
			 	aTEAProcess* pProcess,
			 	tSTACK offset,
			 	unsigned char val);
static aErr sTEAvm_StackPutByteAbs(aTEAvm* pVM,
			 	   aTEAProcess* pProcess,
			 	   tSTACK offset,
			 	   unsigned char val);
static aErr sTEAvm_StackGetShort(aTEAvm* pVM,
				 aTEAProcess* pProcess,
				 tSTACK offset,
				 unsigned short* pVal);
static aErr sTEAvm_StackGetShortAbs(aTEAvm* pVM,
				    aTEAProcess* pProcess,
				    tSTACK offset,
				    unsigned short* pVal);
static aErr sTEAvm_StackPutShort(aTEAvm* pVM,
				 aTEAProcess* pProcess,
				 tSTACK offset,
				 unsigned short val);
static aErr sTEAvm_StackPutShortAbs(aTEAvm* pVM,
				    aTEAProcess* pProcess,
				    tSTACK offset,
				    unsigned short val);

static aErr sTEAvm_PushDecString(aTEAvm* pVM,
				 aTEAProcess* pProcess,
				 long val);
				 
static aErr sTEAvm_Kill(aTEAvm* pVM,
			aTEAProcess* pProcess);
static aVMExit aTEA_ValidateLaunchBlock(aTEAvmLaunchBlock* pLB, 
					const aTEAProcessID nPID);
static aErr sTEAvm_InternalPortProc(aTEAvm* pVM,
				    aTEAProcessID pid,
		       		    tADDRESS port,
			     	    aBool bRead,
			     	    char* data,
			     	    tBYTE dataSize,
			     	    void* ref);

/* aTEAProcess* gpProcess = NULL; */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_FreeProcess
 */

aErr sTEAvm_FreeProcess(
  aTEAvm* pVM, 
  aTEAProcess* pProcess
)
{
  aErr teavmErr = aErrNone;
  
  aVALIDTEAVM(pVM);

#if 0
  /* we only free up the code if we allocated it earlier */
  if ((teavmErr == aErrNone) 
      && (pProcess->code != NULL)
      && (!(pProcess->flags & fTEAvmDebug))) {
    aMemFree((char*)pProcess->code);
  }
  pProcess->code = NULL;
#endif

  if (teavmErr == aErrNone)
    aMemPool_Free(pVM->ioLib, pVM->processPool, pProcess, NULL);
  
  return teavmErr;

} /* sTEAvm_FreeProcess */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_ProcessFromID
 */

aTEAProcess* sTEAvm_ProcessFromID(
  aTEAvm* pVM,
  aTEAProcessID pid
)
{
  aTEAProcess* pProcess = pVM->pProcesses;

  while (pProcess && (pProcess->nPID != pid))
    pProcess = pProcess->pNext;
  
  return pProcess;

} /* sTEAvm_ProcessFromID */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_CallbackStackManipulate
 */

void sTEAvm_CallbackStackManipulate(
  aTEAvmLib vmRef,
  aTEAProcessID pid,
  char operation,
  char* pData,
  tBYTE dataSize
)
{
  aErr teavmErr = aErrNone;

  aTEAProcess* pProcess = sTEAvm_ProcessFromID(vmRef, pid);

  if (!pProcess)
    return;

  switch (operation) {

  case kTEAvmStackIOPush:
    aAssert(pData);
    aAssert(pProcess->nPID == pid);
    if ((pProcess->sp + dataSize) > pProcess->stackSize) {
      teavmErr = sTEAvm_Abort(pProcess->pVM, pProcess, 
    			      aVMExitStackUnderflow);
    } else {
      /* copy in the requested bytes */
      aMemCopy(&pProcess->stack[pProcess->sp], pData, dataSize);
      pProcess->sp += dataSize;
    }
    break;

  case kTEAvmStackIOPop:
    aAssert(pData);
    aAssert(!(pProcess->flags & fTEAvmHalted));
    aAssert(pProcess->nPID == pid);
    if (pProcess->sp < dataSize) {
      teavmErr = sTEAvm_Abort(pProcess->pVM, pProcess, 
    			      aVMExitStackUnderflow);
    } else {
      /* copy out the requested bytes */
      aMemCopy(pData, &pProcess->stack[pProcess->sp - dataSize], dataSize);
      pProcess->sp -= dataSize;
    }
    break;

  case kTEAvmSetHalt:
    aAssert(!(pProcess->flags & fTEAvmHalted));
    aAssert(pProcess->nPID == pid);
    pProcess->flags |= fTEAvmHalted;
    break;

  case kTEAvmClearHalt:
    aAssert(pProcess->flags & fTEAvmHalted);
    aAssert(pProcess->nPID == pid);
    pProcess->flags &= ~fTEAvmHalted;
    break;

  default:
    aAssert(0); /* illegal stack operation */
    break;

  } /* switch */

} /* sTEAvm_CallbackStackManipulate */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_Fetch
 */

aErr sTEAvm_Fetch(
  aTEAvm* pVM
)
{
  aErr teavmErr = aErrNone;
  
  aVALIDTEAVM(pVM);
  
  aAssert(pVM->pCurrentProcess);
  aAssert(!(pVM->pCurrentProcess->flags 
  	    & (fTEAvmHalted | fTEAvmDebug)));

  if (teavmErr == aErrNone) {
    aTEAProcess* pProcess = pVM->pCurrentProcess;
    tADDRESS len;

    /* go get the instruction first */
    aAssert(pProcess->fetchProc);
    pProcess->fetchProc(pProcess->nPID,
    			(unsigned short)(pProcess->cp++ 
    		        		 + aTEA_HEADERSIZE), 
    		        &pVM->fetchRegister[0], 
    		        1, 
    		        pProcess->fetchRef);

    /* find the instruction length */
    len = pVM->opLengths[(int)pVM->fetchRegister[0]];
    aAssert(len < 3);
    aAssert(pVM->fetchRegister[0] <= op_EXIT);

    /* if the instruction has data, go get it */
    if (len > 0) {
      pProcess->fetchProc(pProcess->nPID,
    			  (unsigned short)(pProcess->cp 
    		          		   + aTEA_HEADERSIZE), 
    		          &pVM->fetchRegister[1], 
    		          len, 
    		          pProcess->fetchRef);
      pProcess->cp += len;
    }
  }
  
  return teavmErr;

} /* sTEAvm_Fetch */

#define opClearStateFlags	0x0001
#define opPopbv1		0x0002
#define opPopbv2		0x0004
#define opPopsv1		0x0008
#define	opPopsv2		0x0010
#define opGetOffset		0x0020
#define opCheckByteCarry	0x0040
#define opCheckByteBorrow	0x0080
#define opCheckShortCarry	0x0100
#define opCheckShortBorrow	0x0200
#define opPushByteResult	0x0400
#define opPushShortResult	0x0800

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_Execute
 */

aErr sTEAvm_Execute(aTEAvm* pVM,
		    aTEAProcess* pProcess)
{
  aErr teavmErr = aErrNone;
  unsigned short sv1;
  unsigned short sv2 = 0;
  unsigned char bv1;
  unsigned char bv2 = 0;
  tADDRESS addrVal = 0;
  tSTACK stackAbs;
  long temp = 0;
  unsigned char offset = 0;
  unsigned int ops;

  aAssert(pProcess);
  
  aVALIDTEAVM(pVM);
  
  if (teavmErr == aErrNone) {
  
    ops = 0;

    switch (pVM->fetchRegister[0]) {

    case op_PUSHLB:
      /* push one bytes from the code onto the stack */
      teavmErr = sTEAvm_StackPushByte(pVM, pProcess, 
      				      (unsigned char)pVM->fetchRegister[1]);
      break;

    case op_PUSHLS:
      /* push a short from the code onto the stack */
      sv1 = (unsigned short)aUtil_RetrieveShort(&pVM->fetchRegister[1]);
      teavmErr = sTEAvm_StackPushShort(pVM, pProcess, sv1);
      break;

    case op_PUSHMB:
      /* push a byte from a port onto the stack */
      addrVal = aTEA_RetrieveAddress(&pVM->fetchRegister[1]);
      if ((teavmErr == aErrNone) && pVM->portCBProc)
        teavmErr = sTEAvm_InternalPortProc(pVM, pProcess->nPID,
        				   addrVal, aTrue, (char*)&bv1, 
        				   sizeof(tBYTE), 
        				   pVM->portCBRef);
      if (teavmErr == aErrIO)
        teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitIOErr);
      else
        /* some port ops must halt and wait for data */ 
        if (!(pProcess->flags & fTEAvmHalted))
          teavmErr = sTEAvm_StackPushByte(pVM, pProcess, bv1);
      break;

    case op_PUSHMBX:
      /* push a byte from an indexed port onto the stack */
      teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &addrVal);
      if ((teavmErr == aErrNone) && pVM->portCBProc)
        teavmErr = sTEAvm_InternalPortProc(pVM, pProcess->nPID,
        				   addrVal, aTrue, (char*)&bv1, 
        				   sizeof(tBYTE), 
        				   pVM->portCBRef);
      if (teavmErr == aErrIO)
        teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitIOErr);
      else
        /* some port ops must halt and wait for data */ 
        if (!(pProcess->flags & fTEAvmHalted))
          teavmErr = sTEAvm_StackPushByte(pVM, pProcess, bv1);
      break;

    case op_PUSHMS:
      /* push a short from a port onto the stack */
      addrVal = aTEA_RetrieveAddress(&pVM->fetchRegister[1]);
      if ((teavmErr == aErrNone) && pVM->portCBProc)
        teavmErr = sTEAvm_InternalPortProc(pVM, pProcess->nPID,
        				   addrVal, aTrue, (char*)&sv1, 
        				   sizeof(tSHORT), 
        				   pVM->portCBRef);
      if (teavmErr == aErrIO)
        teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitIOErr);
      else
        /* some port ops must halt and wait for data */ 
        if (!(pProcess->flags & fTEAvmHalted))
          teavmErr = sTEAvm_StackPushShort(pVM, pProcess, sv1);
      break;

    case op_PUSHMSX:
      /* push a short from an indexed port onto the stack */
      teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &addrVal);
      if ((teavmErr == aErrNone) && pVM->portCBProc)
        teavmErr = sTEAvm_InternalPortProc(pVM, pProcess->nPID,
        				   addrVal, aTrue, (char*)&sv1, 
        				   sizeof(tSHORT), 
        				   pVM->portCBRef);
      if (teavmErr == aErrIO)
        teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitIOErr);
      else 
        /* some port ops must halt and wait for data */ 
        if (!(pProcess->flags & fTEAvmHalted))
          teavmErr = sTEAvm_StackPushShort(pVM, pProcess, sv1);
      break;

    case op_PUSHSB:
      teavmErr = sTEAvm_StackGetByte(pVM, pProcess, 
      				     (unsigned char)pVM->fetchRegister[1], &bv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPushByte(pVM,  pProcess, bv1);
      break;

    case op_PUSHSBX:
      teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackGetByte(pVM, pProcess, bv1, &bv2);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPushByte(pVM, pProcess, bv2);
      break;
      
    case op_PUSHSS:
      teavmErr = sTEAvm_StackGetShort(pVM, pProcess,
        			      (unsigned char)pVM->fetchRegister[1], &sv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPushShort(pVM,  pProcess, sv1);
      break;

    case op_PUSHSSX:
      teavmErr = sTEAvm_StackPopOffset(pVM, pProcess, &offset);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackGetShort(pVM, pProcess, offset, &sv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPushShort(pVM, pProcess, sv1);
      break;

    case op_PUSHSBA:
      stackAbs = aTEA_RetrieveStack(&pVM->fetchRegister[1]);
      teavmErr = sTEAvm_StackGetByteAbs(pVM, pProcess, stackAbs, &bv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPushByte(pVM, pProcess, bv1);
      break;

    case op_PUSHSBAX:
      teavmErr = sTEAvm_StackPopAddress(pVM, pProcess, &stackAbs);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackGetByteAbs(pVM, pProcess, 
        				  stackAbs, &bv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPushByte(pVM, pProcess, bv1);
      break;

    case op_PUSHSSA:
      stackAbs = aTEA_RetrieveStack(&pVM->fetchRegister[1]);
      teavmErr = sTEAvm_StackGetShortAbs(pVM, pProcess, stackAbs, &sv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPushShort(pVM, pProcess, sv1);
      break;

    case op_PUSHSSAX:
      teavmErr = sTEAvm_StackPopAddress(pVM, pProcess, &stackAbs);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackGetShortAbs(pVM, pProcess, stackAbs, &sv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPushShort(pVM, pProcess, sv1);
      break;

    case op_PUSHN:
      offset = (unsigned char)pVM->fetchRegister[1];
      if ((pProcess->sp + offset) > pProcess->stackSize)
        teavmErr = sTEAvm_Abort(pVM, pProcess, 
    				aVMExitStackOverflow);
      pProcess->sp += offset;
      break;

    case op_PUSHNX:
      teavmErr = sTEAvm_StackPopOffset(pVM, pProcess, &offset);
      if (teavmErr == aErrNone) {
        if ((pProcess->sp + offset) > pProcess->stackSize)
          teavmErr = sTEAvm_Abort(pVM, pProcess, 
    				aVMExitStackOverflow);
        pProcess->sp += offset;
      }
      break;

    case op_CONVBS:
      teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPushShort(pVM, pProcess, (char)bv1);
      break;

    case op_CONVSB:
      teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &sv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPushByte(pVM, pProcess, (unsigned char)sv1);
      /* set the carry if there was data in the high byte 
       * sort of like an overflow */
      if (((short)sv1 < tBYTE_MIN) || ((short)sv1 > tBYTE_MAX))
        pProcess->stateReg |= fCarry;
      break;

    case op_POPBM:
      addrVal = aTEA_RetrieveAddress(&pVM->fetchRegister[1]);
      teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if ((teavmErr == aErrNone) && pVM->portCBProc)
        teavmErr = sTEAvm_InternalPortProc(pVM, pProcess->nPID,
        				   addrVal, aFalse, (char*)&bv1, 
        				   sizeof(tBYTE), 
        				   pVM->portCBRef);
      if (teavmErr == aErrIO)
        teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitIOErr);
      break;

    case op_POPBMX:
      teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &addrVal);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if ((teavmErr == aErrNone) && pVM->portCBProc)
        teavmErr = sTEAvm_InternalPortProc(pVM, pProcess->nPID,
        				   addrVal, aFalse, (char*)&bv1, 
        				   sizeof(tBYTE), 
        				   pVM->portCBRef);
      if (teavmErr == aErrIO)
        teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitIOErr);
      break;

    case op_POPSM:
      addrVal = aTEA_RetrieveAddress(&pVM->fetchRegister[1]);
      teavmErr = sTEAvm_StackPopShort(pVM, 
      				      pProcess, 
      				      &sv1);
      if ((teavmErr == aErrNone) && pVM->portCBProc) {
        /* flip bytes back */
        sv1 = (unsigned short)(aT2HS(sv1));
        teavmErr = sTEAvm_InternalPortProc(pVM, pProcess->nPID,
        				   addrVal, aFalse, 
        				   (char*)&sv1, 
        				   sizeof(tSHORT), 
        				   pVM->portCBRef);
      }
      if (teavmErr == aErrIO)
        teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitIOErr);
      break;

    case op_POPSMX:
      teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &addrVal);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &sv1);
      if ((teavmErr == aErrNone) && pVM->portCBProc) {
        /* flip bytes back */
        sv1 = (unsigned short)(aT2HS(sv1));
        teavmErr = sTEAvm_InternalPortProc(pVM, pProcess->nPID,
        				   addrVal, aFalse, 
        				   (char*)&sv1, 
        				   sizeof(tSHORT), 
        				   pVM->portCBRef);
      }
      if (teavmErr == aErrIO)
        teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitIOErr);
      break;
    
    case op_POPBS:
      offset = (unsigned char)pVM->fetchRegister[1];
      teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPutByte(pVM, pProcess, offset, bv1);
      break;

    case op_POPBSX:
      teavmErr = sTEAvm_StackPopOffset(pVM, pProcess, &offset);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPutByte(pVM, pProcess, offset, bv1);
      break;

    case op_POPSS:
      offset = (unsigned char)pVM->fetchRegister[1];
      teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &sv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPutShort(pVM, pProcess, offset, sv1);
      break;

    case op_POPSSX:
      teavmErr = sTEAvm_StackPopOffset(pVM, pProcess, &offset);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &sv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPutShort(pVM, pProcess, offset, sv1);
      break;

    case op_POPBSA:
      stackAbs = aTEA_RetrieveStack(&pVM->fetchRegister[1]);
      teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPutByteAbs(pVM, pProcess, stackAbs, bv1);
      break;

    case op_POPBSAX:
      teavmErr = sTEAvm_StackPopAddress(pVM, pProcess, &stackAbs);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPutByteAbs(pVM, pProcess, stackAbs, bv1);
      break;

    case op_POPSSA:
      stackAbs = aTEA_RetrieveStack(&pVM->fetchRegister[1]);
      teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &sv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPutShortAbs(pVM, pProcess, stackAbs, sv1);
      break;

    case op_POPSSAX:
      teavmErr = sTEAvm_StackPopAddress(pVM, pProcess, &stackAbs);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &sv1);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPutShortAbs(pVM, pProcess, stackAbs, sv1);
      break;
    
    case op_POPCMD:
      /* first, get the entire packet size */
      teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if (teavmErr == aErrNone) {
        if (bv1 > pProcess->sp) {
          teavmErr = sTEAvm_Abort(pVM, pProcess, 
    				aVMExitStackUnderflow);
        } else if (bv1 - 2 != pProcess->stack[pProcess->sp - bv1 + 1]) {
          teavmErr = sTEAvm_Abort(pVM, pProcess, 
    				aVMExitBadCommand);
        } else if (bv1 > aSTEMMAXPACKETBYTES) {
          teavmErr = sTEAvm_Abort(pVM, pProcess, 
    				aVMExitBadCommand);
        } else {
          pProcess->sp -= bv1;
          if (pProcess->popCmdProc)
            teavmErr =
              pProcess->popCmdProc(pProcess->nPID,
				   &pProcess->stack[pProcess->sp], 
				   (tBYTE)bv1,
				   pProcess->popCmdRef);
#if 0
          pVM->portCBProc(pProcess->nPID,
        		  aPortCMDOut, aFalse, 
        		  &pProcess->stack[pProcess->sp], 
        		  bv1, pVM->portCBRef);
#endif
        }
      }
      break;

    case op_POPB:
      teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if (teavmErr == aErrNone) {
        pProcess->stateReg &= (unsigned char)~(fZero | fNeg);
        if (bv1 == 0)
          pProcess->stateReg |= fZero;
        else if ((signed char)bv1 < 0)
          pProcess->stateReg |= fNeg;          
      }
      break;

    case op_POPS:
      teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &sv1);
      if (teavmErr == aErrNone) {
        pProcess->stateReg &= (unsigned char)~(fZero | fNeg);
        if (sv1 == 0)
          pProcess->stateReg |= fZero;
        else if ((signed short)sv1 < 0)
          pProcess->stateReg |= fNeg;          
      }
      break;

    case op_POPN:
      offset = (unsigned char)pVM->fetchRegister[1];
      if ((pProcess->sp - offset) < 0)
        teavmErr = sTEAvm_Abort(pVM, pProcess, 
    				aVMExitStackUnderflow);
      pProcess->sp -= offset;
      break;

    case op_POPNX:
      teavmErr = sTEAvm_StackPopOffset(pVM, pProcess, &offset);
      if (teavmErr == aErrNone) {
        if ((pProcess->sp - offset) < 0)
          teavmErr = sTEAvm_Abort(pVM, pProcess, 
    				  aVMExitStackUnderflow);
        pProcess->sp -= offset;
      }
      break;

    case op_DECB:
      /* clear the state bytes for decrement */
      pProcess->stateReg &= (unsigned char)~mStackMask;

      offset = (unsigned char)pVM->fetchRegister[1];

      /* get the operand */
      teavmErr = sTEAvm_StackGetByte(pVM, pProcess, offset, &bv1);

      /* do the math */
      if (teavmErr == aErrNone) {
        temp = bv1 - 1;

        /* set the appropriate bits */
        if ((temp < tBYTE_MIN) 
            || (temp > tBYTE_MAX))
	  pProcess->stateReg |= fCarry;
	else if (temp == 0) {
	  pProcess->stateReg |= fZero;
	}

        /* put the result back on the stack */
        teavmErr = sTEAvm_StackPutByte(pVM, pProcess, offset, (unsigned char)temp);
      }
      break;

    case op_DECS:
      /* clear the state bytes for increment */
      pProcess->stateReg &= (unsigned char)~mStackMask;

      offset = (unsigned char)pVM->fetchRegister[1];

      /* get the operand */
      teavmErr = sTEAvm_StackGetShort(pVM, pProcess, offset, &sv1);

      /* do the math */
      if (teavmErr == aErrNone) {
        temp = sv1 - 1;

        /* set the appropriate bits */
        if ((temp < tSHORT_MIN) 
            || (temp > tSHORT_MAX))
	  pProcess->stateReg |= fCarry;
	else if (temp == 0) {
	  pProcess->stateReg |= fZero;
	}

        /* put the result back on the stack */
        teavmErr = sTEAvm_StackPutShort(pVM, pProcess, offset, (unsigned short)temp);
      }
      break;

    case op_INCB:
      /* clear the state bytes for increment */
      pProcess->stateReg &= (unsigned char)~mStackMask;

      offset = (unsigned char)pVM->fetchRegister[1];

      /* get the operand */
      teavmErr = sTEAvm_StackGetByte(pVM, pProcess, offset, &bv1);

      /* do the math */
      if (teavmErr == aErrNone) {
        temp = bv1 + 1;

        /* set the appropriate bits */
        if ((temp < tBYTE_MIN) 
            || (temp > tBYTE_MAX))
	  pProcess->stateReg |= fCarry;
	else if (temp == 0) {
	  pProcess->stateReg |= fZero;
	}

        /* put the result back on the stack */
        teavmErr = sTEAvm_StackPutByte(pVM, pProcess, offset, (unsigned char)temp);
      }
      break;

    case op_INCS:
      /* clear the state bytes for increment */
      pProcess->stateReg &= (unsigned char)~mStackMask;

      offset = (unsigned char)pVM->fetchRegister[1];

      /* get the operand */
      teavmErr = sTEAvm_StackGetShort(pVM, pProcess, offset, &sv1);

      /* do the math */
      if (teavmErr == aErrNone) {
        temp = sv1 + 1;

        /* set the appropriate bits */
        if ((temp < tSHORT_MIN) 
            || (temp > tSHORT_MAX))
	  pProcess->stateReg |= fCarry;
	else if (temp == 0) {
	  pProcess->stateReg |= fZero;
	}

        /* put the result back on the stack */
        teavmErr = sTEAvm_StackPutShort(pVM, pProcess, offset, (unsigned short)temp);
      }
      break;

    case op_ANDB:
    case op_ORB:
    case op_XORB:
      ops = opClearStateFlags
      	    | opPopbv1
      	    | opPopbv2
      	    | opPushByteResult;
      break;
    
    case op_ANDS:
    case op_ORS:
    case op_XORS:
      ops = opClearStateFlags
      	    | opPopsv1
      	    | opPopsv2
      	    | opPushShortResult;
      break;

    case op_ADDB:
      ops = opClearStateFlags
      	    | opPopbv1
      	    | opPopbv2
      	    | opCheckByteCarry
      	    | opPushByteResult;
      break;

    case op_SUBB:
      ops = opClearStateFlags
      	    | opPopbv1
      	    | opPopbv2
      	    | opCheckByteBorrow
      	    | opPushByteResult;
      break;

    case op_ADDS:
      ops = opClearStateFlags
      	    | opPopsv1
      	    | opPopsv2
      	    | opCheckShortCarry
      	    | opPushShortResult;
      break;

    case op_SUBS:
      ops = opClearStateFlags
      	    | opPopsv1
      	    | opPopsv2
      	    | opCheckShortBorrow
      	    | opPushShortResult;
      break;

    case op_MULTB:
      ops = opClearStateFlags
      	    | opPopbv1
      	    | opPopbv2
      	    | opPushShortResult;
      break;

    case op_DIVB:
    case op_MODB:
      ops = opClearStateFlags
      	    | opPopbv1
      	    | opPopbv2
      	    | opPushByteResult;
      break;

    case op_MULTS:
    case op_DIVS:
    case op_MODS:
      ops = opClearStateFlags
      	    | opPopsv1
      	    | opPopsv2
      	    | opPushShortResult;
      break;

    case op_RRB:
    case op_RLB:
      ops = opCheckByteCarry
            | opClearStateFlags
      	    | opPopbv1
      	    | opPopbv2
      	    | opPushByteResult;
      break;

    case op_NEGB:
    case op_COMPB:
      ops |= opClearStateFlags
      	    | opPopbv1
      	    | opPushByteResult;
      break;

    case op_RRS:
    case op_RLS:
      ops = opCheckShortCarry
            | opClearStateFlags
      	    | opPopbv1
      	    | opPopsv1
      	    | opPushShortResult;
      break;

    case op_NEGS:
    case op_COMPS:
      ops |= opClearStateFlags
      	    | opPopsv1
      	    | opPushShortResult;
      break;

    case op_BRPOS:
      if (!((pProcess->stateReg & fNeg)
            || (pProcess->stateReg & fZero)))
        goto branch;
      break;

    case op_BRNEG:
      if (pProcess->stateReg & fNeg)
        goto branch;
      break;

    case op_BRNZ:
      if (!(pProcess->stateReg & fZero))
        goto branch;
      break;

    case op_BRZ:
      if (pProcess->stateReg & fZero)
        goto branch;
      break;

    case op_BRNC:
      if (!(pProcess->stateReg & fCarry))
        goto branch;
      break;

    case op_BRC:
      if (pProcess->stateReg & fCarry)
        goto branch;
      break;

    case op_CMPBBR:
      teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv2);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if (teavmErr == aErrNone) {
        if (bv1 == bv2)
          goto branch;
        teavmErr = sTEAvm_StackPushByte(pVM, pProcess, bv1);
      }
      break;

    case op_CMPSBR:
      teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &sv2);
      if (teavmErr == aErrNone)
        teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &sv1);
      if (teavmErr == aErrNone) {
        if (sv1 == sv2)
          goto branch;
        teavmErr = sTEAvm_StackPushShort(pVM, pProcess, sv1);
      }
      break;

    case op_GOTO:
branch:
      addrVal = aTEA_RetrieveAddress(&pVM->fetchRegister[1]);
      if (addrVal > pProcess->codeSize)
      	teavmErr = sTEAvm_Abort(pVM, pProcess,
      	  			aVMExitAddressRange);
      pProcess->cp = addrVal;
      break;

    case op_CALL:
      teavmErr = sTEAvm_StackPushAddress(pVM, pProcess, pProcess->cp);
      if (teavmErr == aErrNone) {
        pProcess->cp = aTEA_RetrieveAddress(&pVM->fetchRegister[1]);
      }
      break;

    case op_RETURN:
      teavmErr = sTEAvm_StackPopAddress(pVM, pProcess, &pProcess->cp);
      break;

    case op_EXIT:
      pProcess->eExitCode = aVMExitNormal;
      teavmErr = sTEAvm_Kill(pVM, pProcess);
      break;

    case op_FMTBB:
    case op_FMTBD:
    case op_FMTBU:
    case op_FMTBH:
      ops |= opPopbv1;
      break;

    case op_FMTSB:
    case op_FMTSD:
    case op_FMTSU:
    case op_FMTSH:
      ops |= opPopsv1;
      break;

    } /* switch */

    if ((ops != 0) && (teavmErr == aErrNone)) {

      /* clear the state bytes for operation */
      if ((teavmErr == aErrNone)
 	  && (ops & opClearStateFlags))
        pProcess->stateReg &= (unsigned char)~mStackMask;

      /* bytes must pop before shorts for rotates! */
      if (ops & opPopbv1)
        teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv1);
      if ((teavmErr == aErrNone)
          && (ops & opPopbv2))
        teavmErr = sTEAvm_StackPopByte(pVM, pProcess, &bv2);

      /* bytes must pop before shorts for rotates! */
      if (ops & opPopsv1)
        teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &sv1);
      if ((teavmErr == aErrNone)
          && (ops & opPopsv2))
        teavmErr = sTEAvm_StackPopShort(pVM, pProcess, &sv2);

      switch (pVM->fetchRegister[0]) {

      case op_ADDB:
        temp = bv1 + bv2;
        break;

      case op_ADDS:
        temp = sv1 + sv2;
        break;

      case op_ANDB:
        temp = bv1 & bv2;
        break;

      case op_ANDS:
        temp = sv1 & sv2;
        break;

      case op_SUBB:
        temp = bv2 - bv1;
        break;

      case op_SUBS:
        temp = sv2 - sv1;
        break;
        
      case op_NEGB:
      	temp = -bv1;
      	break;

      case op_NEGS:
      	temp = -sv1;
      	break;
 
      case op_ORB:
        temp = bv1 | bv2;
        break;

      case op_ORS:
        temp = sv1 | sv2;
        break;

      case op_XORB:
        temp = bv1 ^ bv2;
        break;

      case op_XORS:
        temp = sv1 ^ sv2;
        break;
        
      case op_COMPB:
        temp = ~bv1;
        break;

      case op_COMPS:
        temp = ~sv1;
        break;

      case op_MULTB:
        temp = bv1 * bv2;
        break;

      case op_MULTS:
        temp = sv1 * sv2;
        if (temp > tSHORT_MAX)
	  pProcess->stateReg |= fCarry;
	if (temp < tSHORT_MIN)
	  pProcess->stateReg |= fCarry;
        break;

      case op_DIVB:
        if (bv1 == 0)
          teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitDivByZero);
        else
          /* division is signed */
          temp = (char)bv2 / (char)bv1;
        break;

      case op_DIVS:
        if (sv1 == 0)
          teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitDivByZero);
        else
          /* division is signed */
          temp = (short)sv2 / (short)sv1;
        break;

      case op_MODB:
        if (bv1 == 0)
          teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitDivByZero);
        else
          /* mod is signed */
          temp = (char)bv2 % (char)bv1;
        break;

      case op_MODS:
        if (sv1 == 0)
          teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitDivByZero);
        else
          /* mod is signed */
          temp = (short)sv2 % (short)sv1;
        break;
      
      case op_RRB:
        /* rotates are signed */
        temp = (char)bv2 >> bv1;
        break;

      case op_RLB:
        /* rotates are signed */
        temp = (char)bv2 << bv1;
        break;

      case op_RRS:
        /* rotates are signed */      
        temp = (short)sv1 >> bv1;
        break;

      case op_RLS:
        /* rotates are signed */
        temp = (short)sv1 << bv1;
        break;

      case op_POPB:
        if ((signed char)bv1 < 0)
          pProcess->stateReg |= fNeg;
        else if (bv1 == 0)
          pProcess->stateReg |= fZero;
      	break;
      
      case op_POPS:
        if ((signed short)sv1 < 0)
          pProcess->stateReg |= fNeg;
        else if (sv1 == 0)
          pProcess->stateReg |= fZero;
      	break;

      case op_FMTBB:
        bv2 = (unsigned char)0x80;
        if (pProcess->sp + 9 > pProcess->stackSize)
          teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackOverflow);
        else {
          for (temp = 0; temp < 8; temp++) {
            pProcess->stack[pProcess->sp++] = (tBYTE)((bv1 & bv2) ? '1' : '0');
            bv2 = (unsigned char)((bv2 >> 1) & 0x7F);
          }
          pProcess->stack[pProcess->sp++] = 8;
        } 
        break;

      case op_FMTBD:
        teavmErr = sTEAvm_PushDecString(pVM, pProcess, (char)bv1);
        break;

      case op_FMTBU:
        temp = (unsigned char)bv1;
        teavmErr = sTEAvm_PushDecString(pVM, pProcess, temp);
        break;

      case op_FMTBH:
        offset = (unsigned char)bv1;
	teavmErr = sTEAvm_StackPushByte(pVM, pProcess, 
					(unsigned char)(pVM->hexChar[offset >> 4]));
	if (teavmErr == aErrNone)
	  teavmErr = sTEAvm_StackPushByte(pVM, pProcess, 
					  (unsigned char)(pVM->hexChar[offset & 0x0F]));
        temp = 2;
        ops |= opPushByteResult;
        break;

      case op_FMTSB:
        sv2 = (unsigned short)0x8000;
        if (pProcess->sp + 17 > pProcess->stackSize)
          teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackOverflow);
        else {
          for (temp = 0; temp < 16; temp++) {
            pProcess->stack[pProcess->sp++] = (tBYTE)((sv1 & sv2) ? '1' : '0');
            sv2 = (unsigned short)((sv2 >> 1) & 0x7FFF);
          }
          pProcess->stack[pProcess->sp++] = 16;
        } 
        break;

      case op_FMTSD:
        teavmErr = sTEAvm_PushDecString(pVM, pProcess, (short)sv1);
        break;

      case op_FMTSU:
        temp = (unsigned short)sv1;
        teavmErr = sTEAvm_PushDecString(pVM, pProcess, temp);
        break;

      case op_FMTSH:
        addrVal = (tADDRESS)sv1;
	teavmErr = sTEAvm_StackPushByte(pVM, pProcess, 
					(unsigned char)(pVM->hexChar[addrVal >> 12]));
	if (teavmErr == aErrNone)
	  teavmErr = sTEAvm_StackPushByte(pVM, pProcess, 
					  (unsigned char)(pVM->hexChar[(addrVal & 0x0F00) >> 8]));
	if (teavmErr == aErrNone)
	  teavmErr = sTEAvm_StackPushByte(pVM, pProcess, 
					  (unsigned char)(pVM->hexChar[(addrVal & 0x00F0) >> 4]));
	if (teavmErr == aErrNone)
	  teavmErr = sTEAvm_StackPushByte(pVM, pProcess, 
					  (unsigned char)(pVM->hexChar[addrVal & 0x000F]));
        temp = 4;
        ops |= opPushByteResult;
        break;

      } /* switch */

      if ((teavmErr == aErrNone)
          && (ops & opCheckByteCarry)
          && (temp & ~0xFF))
	pProcess->stateReg |= fCarry;

      if ((teavmErr == aErrNone)
          && (ops & opCheckByteBorrow)) {
        if (temp & ~0xFF)
	  pProcess->stateReg &= (unsigned char)~fCarry;
	else
	  pProcess->stateReg |= fCarry;
      }

      if ((teavmErr == aErrNone)
          && (ops & opCheckShortCarry)
          && (temp & ~0xFFFF))
	pProcess->stateReg |= fCarry;

      if ((teavmErr == aErrNone)
          && (ops & opCheckShortBorrow)) {
        if (temp & ~0xFFFF)
	  pProcess->stateReg &= (unsigned char)~fCarry;
	else
	  pProcess->stateReg |= fCarry;
      }
	  

      if ((teavmErr == aErrNone)
          && (ops & opPushByteResult))
        teavmErr = sTEAvm_StackPushByte(pVM, pProcess, (unsigned char)temp);

      if ((teavmErr == aErrNone)
          && (ops & opPushShortResult))
        teavmErr = sTEAvm_StackPushShort(pVM, pProcess, (unsigned short)temp);

    } /* ops */
  }
  
  return teavmErr;

} /* sTEAvm_Execute */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_Abort
 */

aErr sTEAvm_Abort(aTEAvm* pVM, 
		  aTEAProcess* pProcess,
		  aVMExit exitCode)
{
  pProcess->eExitCode = exitCode;
  /* nuke the result if we are in error */
  if (exitCode != aVMExitNormal) {
    aBZero(pProcess->stack, 2);
    pProcess->sp = 0;
  }
  return sTEAvm_Kill(pVM, pProcess);

} /* sTEAvm_Abort */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackPushByte
 */

aErr sTEAvm_StackPushByte(aTEAvm* pVM,
			  aTEAProcess* pProcess,
			  unsigned char val)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);

  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for overflow */
  if ((teavmErr == aErrNone) &&
      (pProcess->sp + sizeof(tBYTE) > pProcess->stackSize))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackOverflow);

  if (teavmErr == aErrNone) {
    pProcess->stack[pProcess->sp++] = (char)val;
    pProcess->stateReg &= (unsigned char)~(fNeg | fZero);
    if ((signed char)val < 0)
      pProcess->stateReg |= fNeg;
    else if (val == 0)
      pProcess->stateReg |= fZero;
  }

  return teavmErr;

} /* sTEAvm_StackPushByte */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackPushShort
 */

aErr sTEAvm_StackPushShort(aTEAvm* pVM,
			   aTEAProcess* pProcess,
			   unsigned short val)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);
  
  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for overflow */
  if ((teavmErr == aErrNone) &&
      (pProcess->sp + sizeof(tSHORT) > pProcess->stackSize)) {
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackOverflow);
  }

  if (teavmErr == aErrNone) {
    aUtil_StoreShort(&pProcess->stack[pProcess->sp], (short)val);
    pProcess->sp += sizeof(tSHORT);
    pProcess->stateReg &= (unsigned char)~(fNeg | fZero);
    if ((short)val < 0)
      pProcess->stateReg |= fNeg;
    else if (val == 0)
      pProcess->stateReg |= fZero;
  }

  return teavmErr;

} /* sTEAvm_StackPushShort */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackPushAddress
 */

aErr sTEAvm_StackPushAddress(aTEAvm* pVM,
			     aTEAProcess* pProcess,
			     tADDRESS val)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);

  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for overflow */
  if ((teavmErr == aErrNone) &&
      (pProcess->sp + sizeof(tADDRESS) > pProcess->stackSize)) {
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackOverflow);
  }

  if (teavmErr == aErrNone) {
    aTEA_StoreAddress(&pProcess->stack[pProcess->sp], val);
    pProcess->sp += sizeof(tSHORT);
  }

  return teavmErr;

} /* sTEAvm_StackPushAddress */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_PushDecString
 */

aErr sTEAvm_PushDecString(aTEAvm* pVM,
			  aTEAProcess* pProcess,
		          long val)
{
  aErr teavmErr = aErrNone;
  char dec[6];
  unsigned int len;

  aStringFromInt(dec, val);
  len = (unsigned int)aStringLen(dec);
  if (pProcess->sp + len + 1 > pProcess->stackSize)
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackOverflow);
  else {
    aMemCopy(&pProcess->stack[pProcess->sp], dec, len);
    pProcess->sp += (tSTACK)len;
    pProcess->stack[pProcess->sp++] = (tBYTE)len;
  }

  return teavmErr;

} /* sTEAvm_PushDecString */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackPopByte
 */

aErr sTEAvm_StackPopByte(aTEAvm* pVM,
		    	 aTEAProcess* pProcess,
		    	 unsigned char* pVal)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);

  aAssert(pVal);
  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for stack underflow */
  if ((teavmErr == aErrNone) && 
      (pProcess->sp < sizeof(tBYTE))) {
    teavmErr = sTEAvm_Abort(pVM, pProcess, 
    			    aVMExitStackUnderflow);
  } else {

    /* do the pop */
    if (teavmErr == aErrNone) {

      *pVal = (unsigned char)pProcess->stack[pProcess->sp - 1];

      /* decrement the stack pointer */
      pProcess->sp -= 1;
    }
  }

  return teavmErr;

} /* sTEAvm_StackPopByte */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackPopShort
 */

aErr sTEAvm_StackPopShort(aTEAvm* pVM,
			  aTEAProcess* pProcess,
			  unsigned short* pVal)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);

  aAssert(pVal);
  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for stack underflow */
  if ((teavmErr == aErrNone) && 
      (pProcess->sp < sizeof(tSHORT))) {
    teavmErr = sTEAvm_Abort(pVM, pProcess,
    			    aVMExitStackUnderflow);
  } else {

    /* do the pop */
    if (teavmErr == aErrNone) {
      pProcess->sp -= sizeof(tSHORT);
      *pVal = (unsigned short)aUtil_RetrieveShort(&pProcess->stack[pProcess->sp]);
    }
  }

  return teavmErr;

} /* sTEAvm_StackPopShort */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackPopAddress
 */

aErr sTEAvm_StackPopAddress(aTEAvm* pVM,
			    aTEAProcess* pProcess,
			    tADDRESS* pVal)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);

  aAssert(pVal);
  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for stack underflow */
  if ((teavmErr == aErrNone) && 
      (pProcess->sp < sizeof(tADDRESS))) {
    teavmErr = sTEAvm_Abort(pVM, pProcess,
    			    aVMExitStackUnderflow);
  }

  /* do the pop */
  if (teavmErr == aErrNone) {
    pProcess->sp -= sizeof(tADDRESS);
    *pVal = aTEA_RetrieveAddress(&pProcess->stack[pProcess->sp]);
  }

  return teavmErr;

} /* sTEAvm_StackPopAddress */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackPopOffset
 */

aErr sTEAvm_StackPopOffset(aTEAvm* pVM,
		    	   aTEAProcess* pProcess,
		    	   tOFFSET* pVal)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);

  aAssert(pVal);
  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for stack underflow */
  if ((teavmErr == aErrNone) && 
      (pProcess->sp < sizeof(tOFFSET))) {
    teavmErr = sTEAvm_Abort(pVM, pProcess, 
    			    aVMExitStackUnderflow);
  } else {

    /* do the pop */
    if (teavmErr == aErrNone) {

      *pVal = (tOFFSET)(pProcess->stack[pProcess->sp - 1]);

      /* decrement the stack pointer */
      pProcess->sp -= sizeof(tOFFSET);
    }
  }

  return teavmErr;

} /* sTEAvm_StackPopOffset */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackGetByte
 */

aErr sTEAvm_StackGetByte(aTEAvm* pVM,
			 aTEAProcess* pProcess,
			 tSTACK offset,
			 unsigned char* pVal)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);
  
  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for underflow */
  if ((teavmErr == aErrNone) && (pProcess->sp < offset))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackError);

  /* check for invalid byte reference (we need 1 byte) */
  if ((teavmErr == aErrNone) && (offset < 1))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackOverflow);

  /* get the actual byte */
  if (teavmErr == aErrNone)
    *pVal = (unsigned char)pProcess->stack[pProcess->sp - offset];

  return teavmErr;

} /* sTEAvm_StackGetByte */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackGetByteAbs
 */

aErr sTEAvm_StackGetByteAbs(aTEAvm* pVM,
			    aTEAProcess* pProcess,
			    tSTACK offset,
			    unsigned char* pVal)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);

  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for overflow */
  if ((teavmErr == aErrNone) && (pProcess->sp <= offset))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackError);

  /* get the actual byte */
  if (teavmErr == aErrNone)
    *pVal = (unsigned char)pProcess->stack[offset];

  return teavmErr;

} /* sTEAvm_StackGetByteAbs */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackPutByte
 */

aErr sTEAvm_StackPutByte(aTEAvm* pVM,
			 aTEAProcess* pProcess,
			 tSTACK offset,
			 unsigned char val)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);
  
  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for underflow */
  if ((teavmErr == aErrNone) && (pProcess->sp < offset))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackError);

  /* check for invalid byte reference (we need 1 byte) */
  if ((teavmErr == aErrNone) && (offset < 1))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackError);

  /* put the actual byte */
  if (teavmErr == aErrNone)
    pProcess->stack[pProcess->sp - offset] = (char)val;

  return teavmErr;

} /* sTEAvm_StackPutByte */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackPutByteAbs
 */

aErr sTEAvm_StackPutByteAbs(aTEAvm* pVM,
			    aTEAProcess* pProcess,
			    tSTACK offset,
			    unsigned char val)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);
  
  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for overflow */
  if ((teavmErr == aErrNone) && (pProcess->sp <= offset))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackError);

  /* put the actual byte */
  if (teavmErr == aErrNone)
    pProcess->stack[offset] = (char)val;

  return teavmErr;

} /* sTEAvm_StackPutByteAbs */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackGetShort
 */

aErr sTEAvm_StackGetShort(aTEAvm* pVM,
			  aTEAProcess* pProcess,
			  tSTACK offset,
			  unsigned short* pVal)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);
  
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for underflow */
  if ((teavmErr == aErrNone) && (pProcess->sp < offset))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackUnderflow);

  /* check for overflow */
  if ((teavmErr == aErrNone) && (offset < 1))
    teavmErr = sTEAvm_Abort(pVM, pProcess,  aVMExitStackOverflow);

  /* check for invalid short reference (we need 2 bytes) */
  if ((teavmErr == aErrNone) && 
      (pProcess->sp < sizeof(tSHORT)))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackUnderflow);

  aAssert(pVal);
  if (teavmErr == aErrNone)
    *pVal = (unsigned short)aUtil_RetrieveShort(&pProcess->stack[pProcess->sp - offset]);

  return teavmErr;

} /* sTEAvm_StackGetShort */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackGetShortAbs
 */

aErr sTEAvm_StackGetShortAbs(aTEAvm* pVM,
			     aTEAProcess* pProcess,
			     tSTACK offset,
			     unsigned short* pVal)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);
  
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for overflow */
  if ((teavmErr == aErrNone) && (pProcess->sp <= offset))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackOverflow);

  /* check for invalid short reference (we need 2 bytes) */
  if ((teavmErr == aErrNone) && (pProcess->sp < sizeof(tSHORT)))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackUnderflow);

  aAssert(pVal);
  if (teavmErr == aErrNone)
    *pVal = (unsigned short)aUtil_RetrieveShort(&pProcess->stack[offset]);

  return teavmErr;

} /* sTEAvm_StackGetShortAbs */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackPutShort
 */

aErr sTEAvm_StackPutShort(aTEAvm* pVM,
			  aTEAProcess* pProcess,
			  tSTACK offset,
			  unsigned short sv)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);
  
  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for underflow */
  if ((teavmErr == aErrNone) && (pProcess->sp < offset))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackError);
  
  /* check for invalid short reference (we need 2 bytes) */
  if ((teavmErr == aErrNone) && (offset < sizeof(tSHORT)))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackError);

  if (teavmErr == aErrNone)
    aUtil_StoreShort(&pProcess->stack[pProcess->sp - offset], 
    		     (short)sv);

  return teavmErr;

} /* sTEAvm_StackPutShort */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_StackPutShortAbs
 */

aErr sTEAvm_StackPutShortAbs(aTEAvm* pVM,
			     aTEAProcess* pProcess,
			     tSTACK offset,
			     unsigned short sv)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);
  
  aAssert(pProcess);
  aAssert(!(pProcess->flags & fTEAvmHalted));

  /* check for stack error */
  if ((teavmErr == aErrNone) 
      && (pProcess->sp < offset + sizeof(tSHORT)))
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitStackError);

  /* store the short */
  if (teavmErr == aErrNone)
    aUtil_StoreShort(&pProcess->stack[offset], (short)sv);

  return teavmErr;

} /* sTEAvm_StackPutShortAbs */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_Kill
 */

aErr sTEAvm_Kill(aTEAvm* pVM,
		 aTEAProcess* pProcess)
{
  aErr teavmErr = aErrNone;

  /* first, remove it from the process list */
  aTEAProcess* pPrev = NULL;
  aTEAProcess* pCur = pVM->pProcesses;
        
  /* handle the case at the head of the list */
  if (pCur == pProcess)
    pVM->pProcesses = pVM->pProcesses->pNext;
  else {
    /* search the list */
    while (pCur != pProcess) {
      pPrev = pCur;
      pCur = pCur->pNext;
      aAssert(pCur);
    }
    pPrev->pNext = pCur->pNext;
  }

  /* execute the exit proc if present */
  if ((teavmErr == aErrNone) &&
      (pProcess->exitProc != NULL)) {
    teavmErr = pProcess->exitProc(pProcess->eExitCode, 
    				  pProcess->stack,
    				  (unsigned char)pProcess->sp,
    				  pProcess->nPID, 
    				  (void*)pProcess->exitRef);
  }
  
  /* now free the process and show its dead */
  if (teavmErr == aErrNone) {
    pVM->nDeadPID = pProcess->nPID;
    teavmErr = sTEAvm_FreeProcess(pVM, pProcess);
  }

  return teavmErr;

} /* sTEAvm_Kill */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvmInternal_Create
 */

aErr aTEAvmInternal_Create(aTEAvm** ppVM)
{
  aErr teavmErr = aErrNone;
  aTEAvm* pVM;
  aIOLib ioLib;

  /* get the IO library object */
  if (teavmErr == aErrNone)
    aIO_GetLibRef(&ioLib, &teavmErr);

  if (teavmErr == aErrNone) {
    pVM = (aTEAvm*)aMemAlloc(sizeof(aTEAvm));
    if (pVM == NULL)
      teavmErr = aErrMemory;
    else {
      aBZero(pVM, sizeof(aTEAvm));
      pVM->check = aTEAVMCHECK;

      /* set up the hex character lookup here because 
       * PalmOS libraries cannot have static data
       */
      pVM->hexChar[0] = '0';
      pVM->hexChar[1] = '1';
      pVM->hexChar[2] = '2';
      pVM->hexChar[3] = '3';
      pVM->hexChar[4] = '4';
      pVM->hexChar[5] = '5';
      pVM->hexChar[6] = '6';
      pVM->hexChar[7] = '7';
      pVM->hexChar[8] = '8';
      pVM->hexChar[9] = '9';
      pVM->hexChar[10] = 'A';
      pVM->hexChar[11] = 'B';
      pVM->hexChar[12] = 'C';
      pVM->hexChar[13] = 'D';
      pVM->hexChar[14] = 'E';
      pVM->hexChar[15] = 'F';

      /* set up the opcode lengths here because 
       * PalmOS libraries cannot have static data
       */
      aTEA_SetOpCodeLengths(pVM->opLengths);

      /* store the io library */
      pVM->ioLib = ioLib;
    }
  }

  if (teavmErr == aErrNone)
    *ppVM = pVM;

  return teavmErr;

} /* aTEAvmInternal_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvmInternal_Initialize
 */

aErr aTEAvmInternal_Initialize(aTEAvm* pVM,
			       unsigned int nMaxStackSize,
			       unsigned int nMaxProcesses,
			       aTEAVMIOPortProc portCBProc,
			       void* portCBRef)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);

  if ((teavmErr == aErrNone) &&
      (pVM->flags & fTEAvmInited))
    teavmErr = aErrInitialization;

  if ((teavmErr == aErrNone) 
      && ((nMaxStackSize > tSTACK_MAX) 
          || (nMaxStackSize < tSTACK_MIN)))
    teavmErr = aErrParam;

  if (teavmErr == aErrNone)
    aMemPool_Create(pVM->ioLib, (aMemSize)(sizeof(aTEAProcess) + nMaxStackSize), 
    		    (aMemSize)nMaxProcesses, &pVM->processPool, &teavmErr);
 
  if (teavmErr == aErrNone) {
    pVM->nMaxStackSize = (tSTACK)nMaxStackSize;
    pVM->nMaxProcesses = nMaxProcesses;
    pVM->nNextPID = 1;
    pVM->portCBProc = portCBProc;
    pVM->portCBRef = portCBRef;
    pVM->flags |= fTEAvmInited;
  }

  return teavmErr;

} /* aTEAvmInternal_Initialize */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvmInternal_TimeSlice
 */

aErr aTEAvmInternal_TimeSlice(aTEAvm* pVM,
			      aBool* pbProcessed)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);

  if ((teavmErr == aErrNone) &&
      !(pVM->flags & fTEAvmInited))
    teavmErr = aErrInitialization;

  if (teavmErr == aErrNone) {

    /* wrap back to head of list */
    if (pVM->pCurrentProcess == NULL)
      pVM->pCurrentProcess = pVM->pProcesses;
    
    if (pVM->pCurrentProcess) {

      /* check for timer expiration */
      if (pVM->pCurrentProcess->wakeTick != 0) {
        unsigned long cur;
        aIO_GetMSTicks(pVM->ioLib, &cur, &teavmErr);
        if ((teavmErr == aErrNone) &&
            (cur >= pVM->pCurrentProcess->wakeTick)) {
          pVM->pCurrentProcess->wakeTick = 0;
          pVM->pCurrentProcess->flags &= (unsigned char)~fTEAvmHalted;
        }
      }

      if (!(pVM->pCurrentProcess->flags & (fTEAvmHalted | fTEAvmDebug))) {
        sTEAvm_Fetch(pVM);
        sTEAvm_Execute(pVM, pVM->pCurrentProcess);
        if (pbProcessed != NULL)
          *pbProcessed = aTrue;
      }
      pVM->pCurrentProcess = pVM->pCurrentProcess->pNext;
    }
  }

  return teavmErr;

} /* aTEAvmInternal_TimeSlice */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvmInternal_Launch
 */

aErr aTEAvmInternal_Launch(
  aTEAvm* pVM,
  aTEAvmLaunchBlock* pLB
)
{
  aErr teavmErr = aErrNone;
  aTEAProcess* pProcess;
  aVMExit validation;

  aVALIDTEAVM(pVM);

  /* check the state of the VM */
  if ((teavmErr == aErrNone) &&
      !(pVM->flags & fTEAvmInited)) {
    teavmErr = aErrInitialization;
  }

  /* if a process id was requested, make sure it is available */
  if ((teavmErr == aErrNone)
      && (pLB->flags & fTEAvmSetPID)
      && (sTEAvm_ProcessFromID(pVM, pLB->pid) != NULL)) {
    teavmErr = aErrBusy;
  }

  /* allocate the process */
  if (teavmErr == aErrNone) {
    aAssert(pVM->processPool);
    aMemPool_Alloc(pVM->ioLib, pVM->processPool, 
    		   (void**)&pProcess, &teavmErr);
  }

  /* set up the process structure */
  if (teavmErr == aErrNone) {
    aBZero(pProcess, sizeof(aTEAProcess));
    pProcess->pVM = pVM;
    pProcess->stack = (char*)(pProcess + 1);
    pProcess->stackSize = pVM->nMaxStackSize;
    pProcess->flags = pLB->flags;
    pProcess->popCmdProc = pLB->popCmdProc;
    pProcess->popCmdRef = pLB->popCmdRef;
    pProcess->fetchProc = pLB->fetchProc;
    pProcess->fetchRef = pLB->fetchRef;
    pProcess->exitProc = pLB->exitProc;
    pProcess->exitRef = pLB->exitRef;

    /* now, handle the allocation pid based of flags */
    if (pLB->flags & fTEAvmSetPID)
      pProcess->nPID = pLB->pid;
    else {
      pProcess->nPID = pVM->nNextPID++;
      pLB->pid = pProcess->nPID;
    }

    /* link it in to the process list */
    pProcess->pNext = pVM->pProcesses;
    pVM->pProcesses = pProcess;
    pVM->nNumProcesses++;
  }

  /* validate the launch block and stop if it doesn't pass */
  if (teavmErr == aErrNone) {
    validation = aTEA_ValidateLaunchBlock(pLB, pProcess->nPID);
    if (validation == aVMExitNormal)
      pProcess->sp = pLB->retValSize;
    else
      return(sTEAvm_Abort(pVM, pProcess, validation));
  }

  /* make sure there is stack space for the data */
  if ((teavmErr == aErrNone)
      && (pVM->nMaxStackSize < pLB->dataSize)) {
    teavmErr = sTEAvm_Abort(pVM, pProcess, aVMExitBadStartup);
  }

  /* prepare the stack with launch data and return value space */
  if ((teavmErr == aErrNone)
       && (pLB->data != NULL) 
       && (pLB->dataSize != 0)) {
    aMemCopy(&pProcess->stack[pProcess->sp], 
      	     pLB->data, pLB->dataSize);
    pProcess->sp += pLB->dataSize;
  }

  /* we need our own copy of the code since the one passed in
   * may be from another shared library
   *
   * we also get rid of the signature bytes here
   */
  if (teavmErr == aErrNone) {
    pProcess->codeSize = (tADDRESS)(pLB->codeSize - aTEA_HEADERSIZE);
    if (pLB->flags & fTEAvmDebug) {
#if 0
     pProcess->code = NULL;
#endif
     pProcess->flags |= fTEAvmDebug;
    } else {
      pProcess->codeSize = (tADDRESS)(pLB->codeSize - aTEA_HEADERSIZE);
#if 0
      pProcess->code = (char*)aMemAlloc((aMemSize)(pLB->codeSize - aTEA_HEADERSIZE));
      if (pProcess->code == NULL) {
        aMemPool_Free(pVM->ioLib, pVM->processPool, pProcess, NULL);
        teavmErr = aErrMemory;
      } else {
        aMemCopy((char*)pProcess->code, &pLB->code[aTEA_HEADERSIZE], 
        	 (aMemSize)(pLB->codeSize - aTEA_HEADERSIZE));
      }
#else
      pProcess->fetchProc = pLB->fetchProc;
      pProcess->fetchRef = pLB->fetchRef;
#endif
    }
  }

  return teavmErr;

} /* aTEAvmInternal_Launch */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvmInternal_Step
 */

aErr aTEAvmInternal_Step(aTEAvm* pVM, 
			 aTEAvmStepBlock* pSB)
{
  aErr teavmErr = aErrNone;
  aTEAProcess* pProcess;

  /* find the process in the process list */
  pProcess = sTEAvm_ProcessFromID(pVM, pSB->pid);

  if (pProcess == NULL)
    teavmErr = aErrParam;

  /* fill the fetch register with the passed in operations */
  if (teavmErr == aErrNone) {
    tADDRESS len;
    pVM->fetchRegister[0] = pSB->op[0];
    len = pVM->opLengths[(int)pVM->fetchRegister[0]];
    aAssert(len < 3);
    aAssert(pVM->fetchRegister[0] <= op_EXIT);
    if (len > 0) {
      aMemCopy(&pVM->fetchRegister[1], 
      	       &pSB->op[1], len);
    }
    pProcess->cp += (tADDRESS)(len + 1);
  }

  /* execute the instruction */
  if (teavmErr == aErrNone) {
    pVM->nDeadPID = 0; /* set to invalid to see if process dies */
    teavmErr = sTEAvm_Execute(pVM, pProcess);
  }

  /* supply the return values */
  if (teavmErr == aErrNone) {
    if (pVM->nDeadPID != 0) {
      pSB->processState = (unsigned char)fPStateExited;
      pSB->pc = 0;
      pSB->sp = 0;
      pSB->stateReg = 0;
    } else {
      pSB->processState = 0;
      pSB->pc = pProcess->cp;
      pSB->sp = pProcess->sp;
      pSB->stateReg = pProcess->stateReg;
    }
  }

  return teavmErr;

} /* aTEAvmInternal_Step */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvmInternal_ViewStack
 */

aErr aTEAvmInternal_ViewStack(aTEAvm* pVM, 
			      const aTEAProcessID pid, 
			      const unsigned char nMode,
			      const tSTACK nStackOffset,
			      const unsigned char nBytes,
			      char* pData)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);
  if ((teavmErr == aErrNone) && (!pData 
  				 || (nMode > 2) 
  				 || (nBytes > 6)
  				 || (nBytes == 0)))
    teavmErr = aErrParam;

  if (teavmErr == aErrNone) {
    aTEAProcess* pProcess = sTEAvm_ProcessFromID(pVM, pid);
    if (pProcess == NULL)
      teavmErr = aErrParam;
    else {
      tSTACK offset = 0;
      
      /* compute the offset based on the access mode */
      switch (nMode) {
      
      /* one-based, from the top of the stack */
      case 0:
        if (((pProcess->sp - nBytes) < 0) || (nStackOffset < nBytes))
          teavmErr = aErrRange;
        else
          offset = (tSTACK)(pProcess->sp - nStackOffset);
        break;

      /* zero-based, from the bottom of the stack */
      case 1:
        if ((nStackOffset + nBytes) > pProcess->stackSize)
          teavmErr = aErrRange;
        else
          offset = nStackOffset;
        break;

      /* checksum mode */
      case 2:
        {
          unsigned char checksum = 0;
          int i;
          for (i = 0; i < pProcess->sp; i++)
            checksum += (unsigned char)pProcess->stack[i];
          *pData = (char)checksum;
        }
        break;

      } /* switch */

      if ((teavmErr == aErrNone) && (nMode != 2))
        aMemCopy(pData, &pProcess->stack[offset], nBytes);
    }
  }

  return teavmErr;

} /* aTEAvmInternal_ViewStack */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvmInternal_Kill
 */

aErr aTEAvmInternal_Kill(aTEAvm* pVM, 
		         aTEAProcessID pid)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);

  if (teavmErr == aErrNone) {
    aTEAProcess* pProcess = sTEAvm_ProcessFromID(pVM, pid);
    if (pProcess == NULL)
      teavmErr = aErrParam;
    else {
      pProcess->eExitCode = aVMExitKill;
      teavmErr = sTEAvm_Kill(pVM, pProcess);
    }
  }

  return teavmErr;

} /* aTEAvmInternal_Kill */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvmInternal_Shutdown
 */

aErr aTEAvmInternal_Shutdown(aTEAvm* pVM)
{
  aErr teavmErr = aErrNone;

  aVALIDTEAVM(pVM);

  /* clean up the current processes */
  if ((teavmErr == aErrNone) &&
      (pVM->flags & fTEAvmInited)) {
    while(pVM->pProcesses) {
      aTEAProcess* pTemp = pVM->pProcesses->pNext;
      sTEAvm_FreeProcess(pVM, pVM->pProcesses);
      pVM->pProcesses = pTemp;
      pVM->nNumProcesses--;
    }
  }

  return teavmErr;

} /* aTEAvmInternal_Shutdown */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvmInternal_Destroy
 */

aErr aTEAvmInternal_Destroy(aTEAvm* pVM)
{
  aErr teavmErr = aErrNone;
  
  aVALIDTEAVM(pVM);
  
  if ((teavmErr == aErrNone) && (pVM->processPool != NULL)) {
    aMemPool_Destroy(pVM->ioLib, pVM->processPool, &teavmErr);
    pVM->processPool = NULL;
  }

  /* release the library reference */  
  if (pVM->ioLib)
    aIO_ReleaseLibRef(pVM->ioLib, NULL);

  if (teavmErr == aErrNone)
    aMemFree((aMemPtr)pVM);

  return teavmErr;

} /* aTEAvmInternal_Destroy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEA_ValidateLaunchBlock
 *
 * Ensure a proper signature for code passed in.  Also determines
 * the return value size and sets it in the block for non-debug 
 * code.
 */

aVMExit aTEA_ValidateLaunchBlock(
  aTEAvmLaunchBlock* pLB,	
  const aTEAProcessID nPID
)
{
  char sig[6];

  aAssert(pLB);

  /* code must be header + single return main
   *  5 bytes is a minimal program built by 
   *  the compiler */
  if (pLB->codeSize < aTEA_HEADERSIZE + 5)
    return aVMExitBadCode;
  
#if 1 
  /* make sure code was passed in */
  if (!pLB->fetchProc)
    return aVMExitBadCode;
#endif

  /* fetch the first few bytes to validate the code signature */
  pLB->fetchProc(nPID,
  		 0, 
    		 sig, 
    		 6, 
    		 pLB->fetchRef);

  /* validate the signature */
  if ((sig[0] != 'a') || (sig[1] != 'T'))
    return aVMExitBadCode;

  /* validate the version */
  if ((sig[2] != aTEA_VERSION_MAJOR)
      || (sig[3] != aTEA_VERSION_MINOR))
    return aVMExitBadVersion;

  /* validate the data size */
  if (sig[5] != pLB->dataSize)
    return aVMExitBadCode;

  /* set the return value size from the header */   
  pLB->retValSize = sig[4];

  return aVMExitNormal;

} /* aTEA_ValidateLaunchBlock */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEAvm_InternalPortProc
 *
 * Handler for I/O port access.
 */

aErr sTEAvm_InternalPortProc(
  aTEAvm* pVM,
  aTEAProcessID pid,
  tADDRESS port,
  aBool bRead,
  char* data,
  tBYTE dataSize,
  void* ref
)
{
  aErr teavmErr = aErrNone;
  aTEAProcess* pProcess;
  
  pProcess = sTEAvm_ProcessFromID(pVM, pid);
  aAssert(pProcess);
  
  if (bRead) {
  
    switch (port) {
    
    default:
      teavmErr = pVM->portCBProc(pid, port, bRead, data, dataSize, 
				 sTEAvm_CallbackStackManipulate, ref);
      break;

    } /* port switch */

  /* else, write */
  } else {

    switch (port) {

    case aPortVMTimer:

      /* if API or not Symonym, we fake a delay */
      if ((pProcess->flags & fTEAvmRobotAPI) ||
          !(pProcess->flags & fTEAvmSymonym)) {
        unsigned long cur;
        if (!aIO_GetMSTicks(pVM->ioLib, &cur, &teavmErr)) {
          if (dataSize == 1) {
            pProcess->wakeTick = cur + *((tBYTE*)data) / 10;
          } else if (dataSize == sizeof(tADDRESS)) {
            tADDRESS duration = aTEA_RetrieveAddress(data);
            pProcess->wakeTick = cur + duration / 10;
          }
        }
        pProcess->flags |= fTEAvmHalted;
        break;
      }
      /* else fall through to default */

    default:
      teavmErr = pVM->portCBProc(pid, port, bRead, data, dataSize, 
				 sTEAvm_CallbackStackManipulate, ref);
      break;

    } /* port switch */
  }
  
  return teavmErr;

} /* sTEAvm_InternalPortProc */	 
