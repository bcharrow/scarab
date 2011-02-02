/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aMotion.c                                                 */
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

#include "aMotion.h"
#include "aStemCore.h"
#include "aUtil.h"
#include "aCmd.tea"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define aMOTIONTIMEOUT	250


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_GetMode
 */

aErr aMotion_GetMode (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  unsigned char* pMode,
  unsigned char* pFlags
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];

  data[0] = cmdMO_CFG;
  data[1] = (char)channel;
  data[2] = 0; /* param index 0 */

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   3, 
    		   data, 
    		   &packet, 
    		   &moErr);

  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);

  /* wait for the reply */
  if (moErr == aErrNone)
    aStem_GetPacket(stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)cmdMO_CFG, 
      		    aMOTIONTIMEOUT, 
      		    &packet, &moErr);

  /* get the reply packet data */
  if (moErr == aErrNone)
    aPacket_GetData(stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    data, 
      		    &moErr);

  /* get the pad value from the packet */
  if (moErr == aErrNone) {
    if ((length == 5)
        && (data[0] == cmdMO_CFG)) {
      *pMode = (unsigned char)data[3];
      *pFlags = (unsigned char)data[4];
    } else {
      moErr = aErrIO;
    }

    /* clean up the packet */
    aPacket_Destroy(stemRef, packet, NULL);
  }

  return moErr;

} /* aMotion_GetMode */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_SetMode
 */

aErr aMotion_SetMode (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const unsigned char val,
  const unsigned char flags
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];

  data[0] = cmdMO_CFG;
  data[1] = (char)channel;
  data[2] = 0; /* param index 0 */
  data[3] = (char)val;
  data[4] = (char)flags;

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   5, 
    		   data, 
    		   &packet, 
    		   &moErr);

  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);    

  return moErr;

} /* aMotion_SetMode */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_GetParam
 */

aErr aMotion_GetParam (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const unsigned char paramIndex,
  short* pVal
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];

  data[0] = cmdMO_CFG;
  data[1] = (char)channel;
  data[2] = (char)paramIndex;

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   3, 
    		   data, 
    		   &packet, 
    		   &moErr);
    
  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);
    
  /* wait for the reply */
  if (moErr == aErrNone)
    aStem_GetPacket(stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)cmdMO_CFG, 
      		    aMOTIONTIMEOUT, 
      		    &packet, &moErr);
    
  /* get the reply packet data */
  if (moErr == aErrNone)
    aPacket_GetData(stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    data, 
      		    &moErr);

  /* get the pad value from the packet */
  if (moErr == aErrNone) {
    if ((length == 5)
        && (data[0] == cmdMO_CFG)) {
      *pVal = aUtil_RetrieveShort(&data[3]);
    } else {
      moErr = aErrIO;
    }

    /* clean up the packet */
    aPacket_Destroy(stemRef, packet, NULL);
  }

  return moErr;

} /* aMotion_GetParam */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_SetParam
 */

aErr aMotion_SetParam (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const unsigned char paramIndex,
  const short val
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];

  data[0] = cmdMO_CFG;
  data[1] = (char)channel;
  data[2] = (char)paramIndex;
  aUtil_StoreShort(&data[3], val);

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   5, 
    		   data, 
    		   &packet, 
    		   &moErr);
    
  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);    

  return moErr;

} /* aMotion_SetParam */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_GetRampParam
 */

aErr aMotion_GetRampParam (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const unsigned char paramIndex,
  short* pVal
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];

  data[0] = cmdMO_RMPCFG;
  data[1] = (char)channel;
  data[2] = (char)paramIndex;

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   3, 
    		   data, 
    		   &packet, 
    		   &moErr);
    
  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);
    
  /* wait for the reply */
  if (moErr == aErrNone)
    aStem_GetPacket(stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)cmdMO_RMPCFG, 
      		    aMOTIONTIMEOUT, 
      		    &packet, &moErr);
    
  /* get the reply packet data */
  if (moErr == aErrNone)
    aPacket_GetData(stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    data, 
      		    &moErr);

  /* get the pad value from the packet */
  if (moErr == aErrNone) {
    if ((length == 5)
        && (data[0] == cmdMO_RMPCFG)) {
      *pVal = aUtil_RetrieveShort(&data[3]);
    } else {
      moErr = aErrIO;
    }

    /* clean up the packet */
    aPacket_Destroy(stemRef, packet, NULL);
  }

  return moErr;

} /* aMotion_GetRampParam */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_SetRampParam
 */

aErr aMotion_SetRampParam (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const unsigned char paramIndex,
  const short val
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];

  data[0] = cmdMO_RMPCFG;
  data[1] = (char)channel;
  data[2] = (char)paramIndex;
  aUtil_StoreShort(&data[3], val);

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   5, 
    		   data, 
    		   &packet, 
    		   &moErr);
    
  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);    

  return moErr;

} /* aMotion_SetRampParam */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_SaveParams
 */

aErr aMotion_SaveParams (
  aStemLib stemRef,
  const unsigned char module,
  const int nChannel
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char len = 1;

  data[0] = cmdMO_SAV;
  if (nChannel != aMOTION_CHANNEL_ALL) {
    data[1] = (char)nChannel;
    len++;
  }

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   len, 
    		   data, 
    		   &packet, 
    		   &moErr);
    
  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);    

  return moErr;

} /* aMotion_SaveParams */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_SetValue
 */

aErr aMotion_SetValue (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const short position
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];

  data[0] = cmdMO_SET;
  data[1] = (char)channel;
  aUtil_StoreShort(&data[2], position);

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   4, 
    		   data, 
    		   &packet, 
    		   &moErr);
    
  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);

  return moErr;

} /* aMotion_SetValue */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_GetValue
 */

aErr aMotion_GetValue (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  short* pPosition
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];

  data[0] = cmdMO_SET;
  data[1] = (char)channel;

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   2, 
    		   data, 
    		   &packet, 
    		   &moErr);
    
  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);
    
  /* wait for the reply */
  if (moErr == aErrNone)
    aStem_GetPacket(stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)cmdMO_SET, 
      		    aMOTIONTIMEOUT, 
      		    &packet, &moErr);
    
  /* get the reply packet data */
  if (moErr == aErrNone)
    aPacket_GetData(stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    data, 
      		    &moErr);

  /* get the pad value from the packet */
  if (moErr == aErrNone) {
    if ((length == 4)
        && (data[0] == cmdMO_SET)) {
      *pPosition = aUtil_RetrieveShort(&data[2]);
    } else {
      moErr = aErrIO;
    }

    /* clean up the packet */
    aPacket_Destroy(stemRef, packet, NULL);
  }

  return moErr;

} /* aMotion_GetValue */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_GetEnc32
 */

aErr aMotion_GetEnc32 (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  char* pbuff
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];

  data[0] = cmdMO_ENC32;
  data[1] = (char)channel;

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   2, 
    		   data, 
    		   &packet, 
    		   &moErr);
    
  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);
    
  /* wait for the reply */
  if (moErr == aErrNone)
    aStem_GetPacket(stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)cmdMO_ENC32, 
      		    aMOTIONTIMEOUT, 
      		    &packet, &moErr);
    
  /* get the reply packet data */
  if (moErr == aErrNone)
    aPacket_GetData(stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    data, 
      		    &moErr);

  /* get the 32-bit encoder value from the packet */
  if (moErr == aErrNone) {
    if ((length == 6)
        && (data[0] == cmdMO_ENC32)) {
      aMemCopy(pbuff, &data[2], 4);
    } else {
      moErr = aErrIO;
    }

    /* clean up the packet */
    aPacket_Destroy(stemRef, packet, NULL);
  }

  return moErr;

} /* aMotion_GetEnc32 */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_GetPIDInput
 * 
 * Returns the PID input value using the cmdMO_PEEK command.
 * 
 * This code is based on code contributed by Jerry Tietz.
 */

aErr aMotion_GetPIDInput (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  short* pValue
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];

  data[0] = cmdMO_PEEK;
  data[1] = (char)channel;

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   2, 
    		   data, 
    		   &packet, 
    		   &moErr);
    
  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);
    
  /* wait for the reply */
  if (moErr == aErrNone)
    aStem_GetPacket(stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)cmdMO_PEEK, 
      		    aMOTIONTIMEOUT, 
      		    &packet, &moErr);
    
  /* get the reply packet data */
  if (moErr == aErrNone)
    aPacket_GetData(stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    data, 
      		    &moErr);

  /* get the PID values */
  if (moErr == aErrNone) {
    if ((length == 8)
        && (data[0] == cmdMO_PEEK)
        && (data[1] == channel)) {
      /* here we use this routine to handle byte swapping */
      /* for big/little endian */
      *pValue = aUtil_RetrieveShort(&data[2]);
    } else {
      moErr = aErrIO;
    }

    /* clean up the packet */
    aPacket_Destroy(stemRef, packet, NULL);
  }

  return moErr;

} /* aMotion_GetPIDInput */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_DecodeFrequency
 */

aErr aMotion_DecodeFrequency (
  const short paramVal,
  long* pFrequency
)
{
  aErr moErr = aErrNone;
  
  if (moErr == aErrNone) {
    int high = (paramVal & 0xFF00) >> 8;
    int low = (paramVal & 0xFF);
    float fFrequency = 1.0f;
    if (high)
      fFrequency = (float)(1 << (high << 1));
    fFrequency = (float)(1.0 / (fFrequency * (low + 1) * 0.0000001));
    if (*pFrequency)
      *pFrequency = (long)fFrequency;
  }

  return moErr;

} /* aMotion_DecodeFrequency */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMotion_RampEnable
 */

aErr aMotion_RampEnable (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char flags,
  const unsigned char channel
)
{
  aErr moErr = aErrNone;
  aPacketRef packet;
  unsigned char size = 2;
  char data[aSTEMMAXPACKETBYTES];

  data[0] = cmdMO_RMPENA;
  data[1] = (char)flags;

  if (channel != aMOTION_RXALL) {
    data[2] = (char)channel;
    size = 3;
  }

  /* build up the packet */
  if (moErr == aErrNone)
    aPacket_Create(stemRef, 
    		   module, 
    		   size, 
    		   data, 
    		   &packet, 
    		   &moErr);
    
  /* send it */
  if (moErr == aErrNone)
    aStem_SendPacket(stemRef, packet, &moErr);

  return moErr;

} /* aMotion_RampEnable */
