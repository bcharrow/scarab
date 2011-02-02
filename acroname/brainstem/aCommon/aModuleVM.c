/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aModuleVM.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of BrainStem module vm access routines. */
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

#include "aCmd.tea"
#include "aStemMsg.h"
#include "aUtil.h"
#include "aModuleVM.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local defines
 */

#define MAXEXECUTETIMEOUT	1000


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes 
 */

static aBool sReturnFilter(const unsigned char module,
		           const unsigned char dataLength,
		           const char* data,
		           void* ref);
static aBool sLaunchFilter(const unsigned char module,
		           const unsigned char dataLength,
		           const char* data,
		           void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sReturnFilter
 */

aBool sReturnFilter(const unsigned char module,
		    const unsigned char dataLength,
		    const char* data,
		    void* ref)
{
  int *pVal = (int*)ref;

  /* if we have a vm exit packet, get the data */
  if ((dataLength >= 4) 
      && ((unsigned char)data[0] == cmdMSG)
      && (data[1] == vmExit)) {
  
    /* get the return value based on the data length */
    switch (dataLength) {
    
    /* char return value (in TEA) */
    case 5:
      *pVal = data[4];
      break;

    /* int return value (in TEA) */
    case 6:
      *pVal = aUtil_RetrieveShort(&data[4]);
      break;

    default:
      *pVal = 0;
      break;

    } /* switch */
    return aTrue;
  }

  return aFalse;

} /* sReturnFilter */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLaunchFilter
 */

aBool sLaunchFilter(const unsigned char module,
		     	   const unsigned char dataLength,
		      	   const char* data,
		      	   void* ref)
{
  int temp = (int)(long)ref;
  if ((data[0] == cmdVM_RUN)
      && (module == (unsigned char)temp)) {
    return aTrue;
  }

  return aFalse;

} /* sLaunchFilter */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aModuleVM_LaunchProcess
 */

aErr aModuleVM_LaunchProcess(aStemLib stemRef,
			     const unsigned char module,
			     const unsigned char fileSlot,
			     const char* pProgramData,
			     const unsigned char nProgramDataLen,
			     const unsigned char flags,
			     aTEAProcessID* pProcID)
{
  aErr moduleErr = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char dataLen;
  char* p;
  unsigned char dataLeft = nProgramDataLen;
  int numPackets = 0;
  aTEAProcessID procid = 0;
  aBool bDone = aFalse;

  data[0] = cmdVM_RUN;
  data[1] = 0;
  p = (char*)pProgramData;

  while ((moduleErr == aErrNone)
         && (bDone != aTrue)) {

    /* clear out the flag byte */
    data[1] = 0;
    length = 2;

    if (numPackets == 0) {
      /* set the first packet bit */
      data[1] |= bitVM_RUN_FIRST;
      
      /* set the file slot */
      data[2] = (char)fileSlot;
      length++;

      /* set the process ID if specified by user */
      if (flags & bitVM_RUN_PID) {
        data[1] |= bitVM_RUN_PID;
        data[3] = (char)*pProcID;
        length++;
      } 

    } else {

      /* packets beyond first need process id */
      data[2] = (char)procid;
      length++;
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
      aMemCopy(&data[length], p, dataLen);
      p += dataLen;
      length += dataLen;
    }
 
    if (dataLeft == 0) {
      data[1] |= bitVM_RUN_LAST;
      bDone = aTrue;
    }

    /* now, build up the packet */
    aPacket_Create(stemRef, module, length, data, &packet, &moduleErr);
    
    /* send it */
    if (moduleErr == aErrNone)
      aStem_SendPacket(stemRef, packet, &moduleErr);
    
    /* wait for the reply */
    if (moduleErr == aErrNone) {
      /* also update number of packets sent */
      numPackets++;
      aStem_GetPacket(stemRef, sLaunchFilter, (void*)((int)module), 
      		      MAXEXECUTETIMEOUT, &packet, &moduleErr);
    }

    /* get the reply packet data */
    if (moduleErr == aErrNone)
      aPacket_GetData(stemRef, packet, &address,
      		      &length, data, &moduleErr);

    /* get the procid from the packet */
    if (moduleErr == aErrNone) {
      aAssert(length == 2);
      aAssert(data[0] == cmdVM_RUN);
      procid = (unsigned char)data[1];
    }
    
    /* clean up the packet */
    if (moduleErr == aErrNone)
      aPacket_Destroy(stemRef, packet, &moduleErr);
  }

  return moduleErr;
  
} /* aModuleVM_LaunchProcess */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aModuleVM_Launch
 */

aErr aModuleVM_Launch(aStemLib stemRef,
		       const unsigned char module,
		       const unsigned char slot,
		       const unsigned char nParamBytes,
		       const char* pParamBytes)
{
  aErr moduleErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
  
  /********************************************/
  /* Use this routine to launch a TEA process */
  /* that takes 0 to 5 input bytes.  Process  */
  /* slot will be assigned automatically.     */
  /********************************************/

  if (!stemRef)
    moduleErr = aErrParam;
  else if (nParamBytes > 5)
    moduleErr = aErrRange;

  /* prepare the packet data */
  if (moduleErr == aErrNone) {
    data[0] = cmdVM_RUN;
    data[1] = 3; /* first with no debug */
    data[2] = (char)slot;
    if (nParamBytes > 0)
      aMemCopy(&data[3], pParamBytes, nParamBytes);
  }

  /* build the packet */
  if (moduleErr == aErrNone) {
    aPacket_Create(stemRef, module, 
    		   (unsigned char)(3 + nParamBytes),
    		   data, &packet, &moduleErr);
  }

  /* send the packet */
  if (moduleErr == aErrNone) {
    aStem_SendPacket(stemRef,
    		     packet,
    		     &moduleErr);
  }

  /* now, wait for launch message */
  if (moduleErr == aErrNone) {
    int temp = module;
    aStem_GetPacket(stemRef,
    		    sLaunchFilter,
    		    (void*)temp,
    		    MAXEXECUTETIMEOUT,
    		    &packet,
    		    &moduleErr);
  }

  /* clean up */
  if (moduleErr == aErrNone) {
    aPacket_Destroy(stemRef, packet, NULL);
  }

  return moduleErr;

} /* aModuleVM_Launch */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aModuleVM_WaitForExit
 */

aErr aModuleVM_WaitForExit(aStemLib stemRef,
			   int* pReturnBytes,
			   int* pReturnValue)
{
  aErr moduleErr = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];

  /************************************/
  /* Use this routine to wait for a   */
  /* TEA process to finish execution  */
  /* and retrieve its return data.    */
  /************************************/

  /* wait for process exit packet */
  if (moduleErr == aErrNone) {
    aStem_GetPacket(stemRef,
    		    sReturnFilter,
    		    pReturnValue,
    		    MAXEXECUTETIMEOUT,
    		    &packet,
    		    &moduleErr);

    /* get the reply packet data */
    if (moduleErr == aErrNone)
      aPacket_GetData(stemRef, packet, &address,
      		      &length, data, &moduleErr);

    /* subtract packet overhead */
    *pReturnBytes = length - 4;
  }

  /* clean up */
  if (moduleErr == aErrNone) {
    aPacket_Destroy(stemRef, packet, NULL);
  }

  return moduleErr;

} /* aModuleVM_WaitForExit */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aModuleVM_Execute
 */

aErr aModuleVM_Execute(aStemLib stemRef,
		       const unsigned char module,
		       const unsigned char slot,
		       const unsigned char nParamBytes,
		       const char* pParamBytes,
		       int* pReturn)
{
  aErr moduleErr = aErrNone;
  aPacketRef packet;

  /*********************************************/
  /* Use this routine to launch a TEA process  */
  /* and wait for it to finish execution.  Use */
  /* it for short programs that do tasks that  */
  /* complete within a second.  The task can   */
  /* take 0 to 5 input bytes.  Process slot    */
  /* will be assigned automatically.           */
  /*********************************************/

  if (!stemRef)
    moduleErr = aErrParam;
  else if (nParamBytes > 5)
    moduleErr = aErrRange;

  /* send RUN packet and get reply */
  if (moduleErr == aErrNone) {
    moduleErr = aModuleVM_Launch(stemRef,
				 module,
				 slot,
				 nParamBytes,
				 pParamBytes);
  }

  /* now, wait for process exit packet */
  if (moduleErr == aErrNone) {
    aStem_GetPacket(stemRef,
    		    sReturnFilter,
    		    pReturn,
    		    MAXEXECUTETIMEOUT,
    		    &packet,
    		    &moduleErr);
  }

  /* clean up */
  if (moduleErr == aErrNone) {
    aPacket_Destroy(stemRef, packet, NULL);
  }

  return moduleErr;

} /* aModuleVM_Execute */
