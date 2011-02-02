/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEADebugExternal.c					   */
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

#include "aCmd.tea"
#include "aConsole.h"
#include "aTEADebugExternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aBool sTEADebugStepFilter(const unsigned char module,
                                 const unsigned char dataLength,
                                 const char* data,
                                 void* ref);
static aBool sTEADebugLaunchFilter(const unsigned char module,
                                   const unsigned char dataLength,
                                   const char* data,
                                   void* ref);
static aBool sTEADebugStackFilter(const unsigned char module,
                                   const unsigned char dataLength,
                                   const char* data,
                                   void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEADebugStepFilter
 */

aBool sTEADebugStepFilter(const unsigned char module,
		     	 const unsigned char dataLength,
		      	 const char* data,
		      	 void* ref)
{
  if (data[0] == cmdDBG_STEP && data[1] == (int)ref)
    return aTrue;

  return aFalse;

} /* sTEADebugStepFilter */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEADebugLaunchFilter
 */

aBool sTEADebugLaunchFilter(const unsigned char module,
		     	    const unsigned char dataLength,
		      	    const char* data,
		      	    void* ref)
{
  if (data[0] == cmdVM_RUN)
    return aTrue;
  
  return aFalse;

} /* sTEADebugLaunchFilter */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEADebugStackFilter
 */

aBool sTEADebugStackFilter(const unsigned char module,
		     	    const unsigned char dataLength,
		      	    const char* data,
		      	    void* ref)
{
  if (data[0] == cmdDBG_STACK && data[1] == (int)ref)
    return aTrue;
  
  return aFalse;

} /* sTEADebugStackFilter */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugExternal_Launch
 */

aErr aTEADebugExternal_Launch(aTEADebugSession* pSession,
			      aTEAvmLaunchBlock* pLB)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole;

  aPacketRef packet;
  unsigned char module;
  unsigned char address;
  unsigned char length;
  char* p;
  char data[aSTEMMAXPACKETBYTES];
  tSTACK dataLen;
  tSTACK dataLeft;
  int numPackets = 0;
  aTEAProcessID pid;

  aAssert(pSession);
  pConsole = (aConsole*)(pSession->pDebugger->vpConsole);
  aAssert(pConsole);

  p = pLB->data;
  dataLeft = pLB->dataSize;
  module = pSession->module;
  data[0] = cmdVM_RUN;

  while ((dsErr == aErrNone)
         && ((numPackets == 0)
             || (dataLeft > 0))) {

    /* clear out the flag byte */
    length = 2;

    if (numPackets == 0) {
      /* set the first packet bit */
      data[1] = bitVM_RUN_FIRST
                     | bitVM_RUN_DEBUG;

      /* store the program size */
      aTEA_StoreAddress(&data[length], pLB->codeSize);
      length += sizeof(tADDRESS);

      /* store the return value size */
      data[length++] = pLB->retValSize;
      data[length++] = (char)pLB->dataSize;

      /* set the process id if passed in */
      if (pLB->flags & fTEAvmSetPID) {
        data[1] |= bitVM_RUN_PID;
        pid = pLB->pid;
        data[length++] = (char)pLB->pid;
      }

    /* packets beyond first need process id */
    } else {
      data[1] = bitVM_RUN_PID;
      data[2] = (char)pid;
      length = 3;
    }

    /* add in any data */
    if (dataLeft > 0) {

      /* stuff as much data as possible */
      dataLen = (unsigned char)(aSTEMMAXPACKETBYTES - length);
      if (dataLeft < dataLen) {
        dataLen = dataLeft;
        dataLeft = 0;
      } else {
        dataLeft -= dataLen;
      }
      
      if (dataLen > 0) {
        aMemCopy(&data[length], p, dataLen);
        p += dataLen;
        length += (unsigned char)dataLen;
      }
    }

    if (dataLeft == 0)
      data[1] |= bitVM_RUN_LAST;
  
    /* now, build up the packet */
    aPacket_Create(pConsole->stemLib, module,
  		     length, data, &packet, &dsErr);
  
    /* send it */
    if (dsErr == aErrNone)
      aStem_SendPacket(pConsole->stemLib, packet, &dsErr);
  
    /* wait for the reply */
    if (dsErr == aErrNone)
      aStem_GetPacket(pConsole->stemLib,
    			sTEADebugLaunchFilter,
    			NULL,
    			pConsole->nMSTimeout,
    			&packet, &dsErr);

    /* get the reply packet data */
    if (dsErr == aErrNone)
      aPacket_GetData(pConsole->stemLib, packet, 
      		&address,
    		        &length, 
    		        data, 
    		        &dsErr);

    if (dsErr == aErrNone) {
      aAssert(length == 2);
      aAssert(data[0] == cmdVM_RUN);
      pid = (aTEAProcessID)data[1];
      numPackets++;
    }

    /* clean up the reply packet */
    if (dsErr == aErrNone)
      aPacket_Destroy(pConsole->stemLib, packet, &dsErr);

  } /* while */

  /* if pid not set, pass it back */
  if (!(pLB->flags & fTEAvmSetPID))
    pLB->pid = pid;
    
  return dsErr;

} /* aTEADebugExternal_Launch */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugExternal_Step
 */

aErr aTEADebugExternal_Step(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)(pSession->pDebugger->vpConsole);
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];
  char* op;

  aAssert(pSession);
  aAssert(pConsole);
  
  op = pSession->pInstr[pSession->nInstrIndex].op;
  length = (unsigned char)(3 + pSession->pDebugger->opLengths[(int)*op]);

  data[0] = cmdDBG_STEP;
  data[1] = (char)sProcessID(pSession);
  aMemCopy(&data[2], op, (unsigned int)(length - 2));

  /* now, build up the packet */
  aPacket_Create(pConsole->stemLib, pSession->module,
  		   length, data, &packet, &dsErr);

  /* send it */
  if (dsErr == aErrNone)
    aStem_SendPacket(pConsole->stemLib, packet, &dsErr);

  /* wait for the reply (30 SECOND TIMEOUT) */
  if (dsErr == aErrNone)
    aStem_GetPacket(pConsole->stemLib,
    		      sTEADebugStepFilter,
    		      (void*)((int)sProcessID(pSession)),
    		      30000,
    		      &packet, &dsErr);

  /* get the reply packet data */
  if (dsErr == aErrNone)
    aPacket_GetData(pConsole->stemLib, packet, 
      	      &address,
    		      &length, 
    		      data, 
    		      &dsErr);

  /* check for exit */
  if (dsErr == aErrNone) {
    if (data[2] & fPStateExited) {
      pSession->SB.processState |= fPStateExited;
      /* set step block PC, SP for normal exit */
      /* (SP has junk on exit from module) */
      pSession->SB.pc = 0;
      pSession->SB.sp = 0;
      pSession->SB.stateReg = 0;
    } else {
      /* set regs for normal return*/
      pSession->SB.pc = aTEA_RetrieveAddress(&data[3]);
      pSession->SB.sp = aTEA_RetrieveAddress(&data[5]);
      pSession->SB.stateReg = (unsigned char)data[7];
    }
  }


  /* clean up the packet */
  if (dsErr == aErrNone)
    aPacket_Destroy(pConsole->stemLib, packet, &dsErr);
  
  return dsErr;

} /* aTEADebugExternal_Step */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugExternal_ViewStack
 */

aErr aTEADebugExternal_ViewStack(aTEADebugSession* pSession,
				 const unsigned char nMode,
				 const tSTACK nStackOffset,
				 const unsigned char nBytes,
				 char* pData)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)(pSession->pDebugger->vpConsole);

  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];

  aAssert(pSession);
  aAssert(pConsole);  


  /* stuff the data */  
  data[0] = cmdDBG_STACK;
  data[1] = (char)sProcessID(pSession);
  data[2] = (char)nMode;
  data[3] = (char)(nStackOffset/256);
  data[4] = (char)(nStackOffset-(data[3]*256));
  data[5] = (char)nBytes;
  length = 6;

  /* now build up the packet */
  aPacket_Create(pConsole->stemLib, pSession->module,
  	   length, data, &packet, &dsErr);

  /* send it */
  if (dsErr == aErrNone)
  aStem_SendPacket(pConsole->stemLib, packet, &dsErr);

  /* wait for the reply */
  if (dsErr == aErrNone)
  aStem_GetPacket(pConsole->stemLib,
		  sTEADebugStackFilter,
		  (void*)((int)sProcessID(pSession)),
		  pConsole->nMSTimeout,
		  &packet, &dsErr);

  /* get the reply packet data */
  if (dsErr == aErrNone)
  aPacket_GetData(pConsole->stemLib, packet, 
                &address,
                &length, 
                data, 
                &dsErr);

  /* copy data bytes to input area (strip header) */
  if (dsErr == aErrNone)
  aMemCopy(pData, &data[2], (unsigned int)(length - 2));


  /* clean up the packet */
  if (dsErr == aErrNone)
  aPacket_Destroy(pConsole->stemLib, packet, &dsErr);

  return dsErr;

} /* aTEADebugExternal_ViewStack */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugExternal_Kill
 */

aErr aTEADebugExternal_Kill(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)(pSession->pDebugger->vpConsole);

  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];

  aAssert(pSession);
  aAssert(pConsole);  

  /* stuff the data */  
  data[0] = cmdVM_KILL;
  data[1] = (char)sProcessID(pSession);
  length = 6;

  /* now build up the packet */
  aPacket_Create(pConsole->stemLib, pSession->module,
  	   length, data, &packet, &dsErr);

  /* send it */
  if (dsErr == aErrNone)
    aStem_SendPacket(pConsole->stemLib, packet, &dsErr);
  
  /* wait for the reply */
  /* (it will be a cmdDBG_STEP packet) */
  if (dsErr == aErrNone)
    aStem_GetPacket(pConsole->stemLib,
		    sTEADebugStepFilter,
		    (void*)((int)sProcessID(pSession)),
		    pConsole->nMSTimeout,
		    &packet, &dsErr);

  /* get the reply packet data */
  if (dsErr == aErrNone)
    aPacket_GetData(pConsole->stemLib, packet, 
                    &address,
                    &length, 
                    data, 
                    &dsErr);

  /* process exits after kill */
  aAssert(data[2] & fPStateExited);
  {
      pSession->SB.processState |= fPStateExited;
      /* set step block PC, SP for normal exit */
      /* (SP has junk on exit from module) */
      pSession->SB.pc = 0;
      pSession->SB.sp = 0;
      pSession->SB.stateReg = 0;
  }

  /* clean up the packet */
  if (dsErr == aErrNone)
    aPacket_Destroy(pConsole->stemLib, packet, &dsErr);

  return dsErr;

} /* aTEADebugExternal_Kill */

#endif /* aDEBUGGER */
