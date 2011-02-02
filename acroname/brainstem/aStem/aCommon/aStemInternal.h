/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStemInternal.h                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent BrainStem	   */
/*		communication layer.    			   */
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

#ifndef _aStemInternal_H_
#define _aStemInternal_H_

#include "aIO.h"
#include "aPacket.h"
#include "aSerialRelayStream.h"


/* computes the address to relay array index */
#define aRELAYINDEX(a) ((a >> 1) - 1)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * constant definitions
 */

#define aSTEMMAXDEBUGLINE		80
#define aSTEMMAXNAMELENGTH		80
#define aPACKETPOOLBLOCKSIZE 		16
#define aSTEMMAXRELAYS                 126


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * synchronization states
 */

#define kSyncStateSynced		0
#define	kSyncStateStart			1
#define kSyncStateByte1			2
#define kSyncStateByte2			3
#define kSyncStateByte3			4
#define kSyncStateByte4			5


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * type definitions
 */

typedef struct aStem {
  aIOLib			ioRef;
  aStreamRef			debugStream;
  aStreamRef			linkStream;
  aStreamRef			relayStream;
  aSettingFileRef		stemSettings;
  int				check;
  aMemPoolRef			packetPoolRef;
  aPacket			curLinkPacket;
  aPacket			curRelayPacket;
  aPacketList*			pLinkPacketList;
  aPacketList*			pRelayPacketList;
  aHeartbeatCallback		heartbeatCB;
  void*				heartbeatCBRef;
  int				nReSyncState;
  char				cSyncCache;
  int				nTEADataDelayMS;
  struct aSerialRelayStream*	relays[aSTEMMAXRELAYS];
  char				sName[aSTEMMAXNAMELENGTH];

} aStem;

#define aSTEMCHECK	0xEEEE

#define aVALIDSTEM(p)							\
  if ((err == aErrNone)							\
      && ((!p) || (((aStem*)p)->check != aSTEMCHECK)))			\
    err = aErrParam;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * routine definitions
 */

aErr 
aStem_DrainStream(aStem* pStem,
		  aStreamRef stream,
		  aPacket* pPacketStore,
		  aPacketFilter filterProc,
		  void* filterRef,
		  aBool bHandleHB,
		  unsigned char initflags,
		  aPacketList* pPacketList);

#endif /* _aStemInternal_H_ */
