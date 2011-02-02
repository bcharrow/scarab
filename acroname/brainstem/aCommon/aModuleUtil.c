/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aModuleUtil.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of BrainStem module utility         */
/*		function routines.                                 */
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
#include "aModuleUtil.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local defines
 */

#define MAXUTILTIMEOUT	500


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes 
 */

static aBool sReturnFilter(const unsigned char module,
		           const unsigned char dataLength,
		           const char* data,
		           void* ref);

static aBool sMagicByteFilter(const unsigned char module,
			      const unsigned char dataLength,
			      const char* data,
			      void* ref);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMagicByteFilter
 */

aBool sMagicByteFilter(
  const unsigned char module,
  const unsigned char dataLength,
  const char* data,
  void* ref)
{
  if ((dataLength == 0) && module && !(module & 0x01))
    return aTrue;

  return aFalse;

} /* aMagicByteFilter */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sReturnFilter
 */

aBool sReturnFilter(const unsigned char module,
		    const unsigned char dataLength,
		    const char* data,
		    void* ref)
{
  /* see if we got our debug packet back */
  if ((dataLength == 2) 
      && ((unsigned char)data[0] == cmdDEBUG)) {
    return aTrue;
  }

  return aFalse;

} /* sReturnFilter */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aModuleUtil_EnsureModule
 *
 * This routine ensures a live module is in place and that the 
 * heartbeat is established with the module.
 */

aBool aModuleUtil_EnsureModule(aStemLib stemRef,
			       const unsigned char address)

{
  aBool bModule = aTrue;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
  int i;

  if (!stemRef)
    bModule = aFalse;

  /* prepare the packet data */
  
  if (bModule) {
    aErr muErr = aErrNone;
    for (i = 0; (muErr == aErrNone) && (i < 3); i++) {

      /* build a debug packet */
      data[0] = cmdDEBUG;
      data[1] = (char)i;
      aPacket_Create(stemRef, address, 2,
    		     data, &packet, &muErr);

      /* send the packet */
      if (muErr == aErrNone)
        aStem_SendPacket(stemRef, packet, &muErr);

      /* now, wait for the reply packet */
      if (muErr == aErrNone)
        aStem_GetPacket(stemRef,
    		        sReturnFilter,
    		        (void*)(long)i,
    		        MAXUTILTIMEOUT,
    		        &packet,
    		        &muErr);

      /* clean up the reply packet */
      if (muErr == aErrNone)
        aPacket_Destroy(stemRef, packet, &muErr);

      if ((i < 2) && (muErr == aErrTimeout))
        muErr = aErrNone;
    } /* for */
    if (muErr != aErrNone)
      bModule = aFalse;
  }

  return bModule;

} /* aModuleUtil_EnsureModule */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aModuleUtil_CheckForModule
 *
 * This routine checks for the presence of a module.
 * The module may or may not be a router or using link heartbeats.
 */

aErr aModuleUtil_CheckForModule(
  aStemLib stemLib,
  unsigned char* pmodule
)
{
  aErr connErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char ucmodule;
  unsigned char uclen;
  aPacketRef packet;

  /* create magic byte command to get address */
  if (connErr == aErrNone)
    aPacket_Create(stemLib,
    		   (unsigned char)cmdMAGIC,
    		   (unsigned char)0,
    		   data,
    		   &packet,
    		   &connErr);

  /* send magic packet */
  if (connErr == aErrNone)
    aStem_SendPacket(stemLib,
    		     packet,
    		     &connErr);

  /* await magic reply */
  if (connErr == aErrNone)
    aStem_GetPacket(
      stemLib, 
      sMagicByteFilter, 
      NULL,
      100, 
      &packet, 
      &connErr);

  /* an error means the connection is bad */      
  if (connErr != aErrNone)
    connErr = aErrConnection;

  /* otherwise we can retrieve the module code */
  if (connErr == aErrNone) {

    aErr bdChkErr = aErrNone;
    aPacket_GetData(
      stemLib, 
      packet, 
      &ucmodule,
      &uclen, 
      data, 
      &bdChkErr);

    if (bdChkErr == aErrNone) {
      /* stash address */
      if (pmodule)
        *pmodule = ucmodule;
      aPacket_Destroy(stemLib, packet, NULL);
    }
  }
  
  return connErr;

} /* aModuleUtil_CheckForModule */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aModuleUtil_ForceLink
 *
 * This routine sends a heartbeat which
 * forces module to send replies back to host
 * but only if the module is routing.
 */

aErr aModuleUtil_ForceLink(
  aStemLib stemLib,
  const unsigned char module
)
{
  aErr pingErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  /* slam the Stem with a heartbeat to force link to be active */
  /* (this is a hack, not an original feature of heartbeat commands) */
  if (pingErr == aErrNone) {
    data[0] = cmdHB;
    data[1] = 3; /* h2s_HB_DOWN */
    if (!aPacket_Create(stemLib, 2, 2, data, &packet, &pingErr))
      aStem_SendPacket(stemLib, packet, &pingErr);
  }
  
  return pingErr;

} /* aModuleUtil_ForceLink */
