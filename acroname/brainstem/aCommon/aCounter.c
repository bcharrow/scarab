/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aCounter.c                                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Routine implementations for accessing the counters */
/*              on the BrainStem.                                  */
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

#include "aCounter.h"
#include "aStemCore.h"
#include "aUtil.h"
#include "aCmd.tea"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define aCOUNTERIOTIMEOUT	50


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aCounter_ReadShort
 */

aErr aCounter_ReadShort(aStemLib stemRef,
		        const unsigned char module,
		        const unsigned char counterIndex,
		        short* psVal)
{
  aErr counterErr = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];
  
  if ((counterErr == aErrNone) && (counterIndex > 7))
    counterErr = aErrRange;

  /* build up the packet */
  if (counterErr == aErrNone) {
    data[0] = cmdCTR_SET;
    data[1] = (char)counterIndex;
    aPacket_Create(stemRef, 
    		   module, 
    		   2, 
    		   data, 
    		   &packet, 
    		   &counterErr);
  }
    
  /* send it */
  if (counterErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &counterErr);
    
  /* wait for the reply */
  if (counterErr == aErrNone)
    aStem_GetPacket(stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)cmdCTR_SET, 
      		    aCOUNTERIOTIMEOUT, 
      		    &packet, &counterErr);

  /* get the reply packet data */
  if (counterErr == aErrNone)
    aPacket_GetData(stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    data, 
      		    &counterErr);

  /* get the pad value from the packet */
  if (counterErr == aErrNone) {
    if ((length == 4)
        && (data[0] == cmdCTR_SET)) {
      *psVal = aUtil_RetrieveShort(&data[2]);
    } else {
      counterErr = aErrIO;
    }

    /* clean up the packet */
    aPacket_Destroy(stemRef, packet, NULL);
  }

 
  return counterErr;

} /* aPad_ReadChar */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aCounter_WriteShort
 */

aErr aCounter_WriteShort(aStemLib stemRef,
		         const unsigned char module,
		         const unsigned char counterIndex,
		         const short sVal)
{
  aErr counterErr = aErrNone;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];

  if ((counterErr == aErrNone) && (counterIndex > 7))
    counterErr = aErrRange;
 
  /* build up the packet */
  if (counterErr == aErrNone) {
    data[0] = cmdCTR_SET;
    data[1] = (char)counterIndex;
    aUtil_StoreShort(&data[2], sVal);
    aPacket_Create(stemRef, 
    		   module, 
    		   4, 
    		   data, 
    		   &packet, 
    		   &counterErr);
  }
    
  /* send it */
  if (counterErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &counterErr);

  return counterErr;

} /* aPad_ReadInt */


