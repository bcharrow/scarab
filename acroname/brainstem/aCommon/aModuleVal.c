/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aModuleVal.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of BrainStem module value access        */
/*              routines.                                          */
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
#include "aStemCore.h"
#include "aUtil.h"
#include "aModuleVal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local defines
 */

#define MAXVALTIMEOUT	500



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aModuleVal_Get
 */

aErr aModuleVal_Get(aStemLib stemRef,
		     const unsigned char module,
		     const char eVal,
		     char* pVal)
{
  aErr mvErr = aErrNone;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  aAssert(pVal);

  if (!stemRef)
    mvErr = aErrParam;

  /* prepare the packet data */
  if (mvErr == aErrNone) {
    data[0] = cmdVAL_GET;
    data[1] = eVal;
    aPacket_Create(stemRef, module, 2,
    		   data, &packet, &mvErr);
  }

  /* send the packet */
  if (mvErr == aErrNone) {
    aStem_SendPacket(stemRef, packet, &mvErr);
  }

  /* now, wait for the reply packet */
  if (mvErr == aErrNone) {
    aStem_GetPacket(stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)cmdVAL, 
      		    MAXVALTIMEOUT, 
      		    &packet,
      		    &mvErr);
  }

  /* get the reply packet data */
  if (mvErr == aErrNone)
    aPacket_GetData(stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    data, 
      		    &mvErr);

  /* get the system value from the packet */
  if (mvErr == aErrNone) {
    if (length == 3) {
      *pVal = data[2];
    } else if (length == 4) {
      pVal[0] = data[2];
      pVal[1] = data[3];
    } else {
      mvErr = aErrIO;
    }
    
    /* clean up the packet */
    aPacket_Destroy(stemRef, packet, NULL);
  }

  return mvErr;

} /* aModuleVal_Get */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aModuleVal_Set
 */

aErr aModuleVal_Set(aStemLib stemRef,
		     const unsigned char module,
		     const char eVal,
		     const char val)
{
  aErr mvErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  if (!stemRef)
    mvErr = aErrParam;

  /* prepare the packet data */
  if (mvErr == aErrNone) {
    data[0] = cmdVAL_SET;
    data[1] = eVal;
    data[2] = val;
    aPacket_Create(stemRef, module, 3,
    		   data, &packet, &mvErr);
  }

  /* send the packet */
  if (mvErr == aErrNone) {
    aStem_SendPacket(stemRef,
    		     packet,
    		     &mvErr);
  }

  return mvErr;

} /* aModuleVal_Set */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aModuleVal_Save
 */

aErr aModuleVal_Save(aStemLib stemRef,
		     const unsigned char module)
{
  aErr mvErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  if (!stemRef)
    mvErr = aErrParam;

  /* prepare the packet data */
  if (mvErr == aErrNone) {
    data[0] = cmdVAL_SAV;
    aPacket_Create(stemRef, module, 1,
    		   data, &packet, &mvErr);
  }

  /* send the packet */
  if (mvErr == aErrNone) {
    aStem_SendPacket(stemRef,
    		     packet,
    		     &mvErr);
  }

  return mvErr;

} /* aModuleVal_Save */
