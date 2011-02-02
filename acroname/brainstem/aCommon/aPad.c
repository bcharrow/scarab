/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aPad.c                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Routine implementations for accessing the scratch  */
/*              pad on the BrainStem.				   */
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

#include "aPad.h"
#include "aStemCore.h"
#include "aUtil.h"
#include "aCmd.tea"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define aPADIOTIMEOUT	500


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPad_ReadChar
 */

aErr aPad_ReadChar(aStemLib stemRef,
			 const unsigned char module,
			 const unsigned char padIndex,
			 char* pVal)
{
  aErr padErr = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];
  
  data[0] = cmdPAD_IO;
  data[1] = (char)padIndex;

  /* build up the packet */
  if (padErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   2, 
    		   data, 
    		   &packet, 
    		   &padErr);
    
  /* send it */
  if (padErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &padErr);
    
  /* wait for the reply */
  if (padErr == aErrNone)
    aStem_GetPacket(stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)cmdPAD_IO, 
      		    aPADIOTIMEOUT, 
      		    &packet, &padErr);
    
  /* get the reply packet data */
  if (padErr == aErrNone)
    aPacket_GetData(stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    data, 
      		    &padErr);

  /* get the pad value from the packet */
  if (padErr == aErrNone) {
    if ((length == 3)
        && (data[0] == cmdPAD_IO)) {
      *pVal = data[2];
    } else {
      padErr = aErrIO;
    }
    
    /* clean up the packet */
    aPacket_Destroy(stemRef, packet, NULL);
  }

 
  return padErr;

} /* aPad_ReadChar */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPad_ReadInt
 */

aErr aPad_ReadInt(aStemLib stemRef,
			 const unsigned char module,
			 const unsigned char padIndex,
			 int* pVal)
{
  aErr padErr = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];
  char val[2];

  data[0] = cmdPAD_IO;
  data[1] = (char)padIndex;

  /* build up the packet */
  if (padErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   2, 
    		   data, 
    		   &packet, 
    		   &padErr);
    
  /* send it */
  if (padErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &padErr);
    
  /* wait for the reply */
  if (padErr == aErrNone)
    aStem_GetPacket(stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)cmdPAD_IO, 
      		    aPADIOTIMEOUT, 
      		    &packet, &padErr);
    
  /* get the reply packet data */
  if (padErr == aErrNone)
    aPacket_GetData(stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    data, 
      		    &padErr);

  /* get the pad value from the packet */
  if (padErr == aErrNone) {
    if ((length == 3)
        && (data[0] == cmdPAD_IO)) {
      val[0] = data[2];
    } else {
      padErr = aErrIO;
    }
    
    /* clean up the packet */
    aPacket_Destroy(stemRef, packet, NULL);
  }

  /* build up the second packet */
  if (padErr == aErrNone)
    data[1]++;
    aPacket_Create(stemRef, 
    		   module, 
    		   2, 
    		   data, 
    		   &packet, 
    		   &padErr);
    
  /* send it */
  if (padErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &padErr);
    
  /* wait for the reply */
  if (padErr == aErrNone)
    aStem_GetPacket(stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)cmdPAD_IO, 
      		    aPADIOTIMEOUT, 
      		    &packet, &padErr);
    
  /* get the reply packet data */
  if (padErr == aErrNone)
    aPacket_GetData(stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    data, 
      		    &padErr);

  /* get the pad value from the packet */
  if (padErr == aErrNone) {
    if ((length == 3)
        && (data[0] == cmdPAD_IO)) {
      val[1] = data[2];
      *pVal = aUtil_RetrieveShort(val);
    } else {
      padErr = aErrIO;
    }
    
    /* clean up the packet */
    aPacket_Destroy(stemRef, packet, NULL);
  }

  return padErr;

} /* aPad_ReadInt */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPad_WriteChar
 */

aErr aPad_WriteChar(aStemLib stemRef,
			 const unsigned char module,
			 const unsigned char padIndex,
			 const char val)
{
  aErr padErr = aErrNone;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];

  /* we write the data one byte at a time */
  data[0] = cmdPAD_IO;
  data[1] = (char)padIndex;
  data[2] = val;

  /* build up the packet */
  if (padErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   3, 
    		   data, 
    		   &packet, 
    		   &padErr);
    
  /* send it */
  if (padErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &padErr);

  return padErr;

} /* aPad_WriteChar */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPad_WriteInt
 */

aErr aPad_WriteInt(aStemLib stemRef,
			 const unsigned char module,
			 const unsigned char padIndex,
			 const int val)
{
  aErr padErr = aErrNone;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];

  /* we write the data one byte at a time */
  data[0] = cmdPAD_IO;
  data[1] = (char)padIndex;
  aUtil_StoreShort(&data[2], (short)val);

  /* build up the packet */
  if (padErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   3, 
    		   data, 
    		   &packet, 
    		   &padErr);
    
  /* send it */
  if (padErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &padErr);

  data[0] = cmdPAD_IO;
  data[1] = (char)(padIndex + 1);
  data[2] = data[3];

  /* build up the packet */
  if (padErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   3, 
    		   data, 
    		   &packet, 
    		   &padErr);
    
  /* send it */
  if (padErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &padErr);

  return padErr;

} /* aPad_WriteInt */
