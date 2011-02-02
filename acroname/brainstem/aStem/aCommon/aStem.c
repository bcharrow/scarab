/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStem.c                                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent BrainStem */
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

#include "aStem.h"
#include "aStemInternal.h"
#include "aStemText.h"
#include "aVersion.h"


#define RAWDEBUG 0

/* define RAWDEBUG to log every byte comming in StemDebug */
#if RAWDEBUG
static void sRAWDEBUG(aStem* pStem, const char byte);
#else /* RAWDEBUG */
#define sRAWDEBUG(s, b)
#endif /* RAWDEBUG */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr 
sStem_LogDebugPacket(aStem* pStem, 
		     aPacket* pPacket);
static aErr 
sStem_HandleBackChannel(aStem* pStem,
			aPacket* pPacket,
			aPacketFilter filterProc,
			void* filterRef,
			aPacketList* pPacketList);
static aErr
sStem_Destroy(aStem* pStem);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem_GetLibRef
 */

aLIBRETURN 
aStem_GetLibRef(aStemLib* pStemRef, 
		aErr* pErr)
{
  return aStem_GetNamedLibRef(pStemRef, "", pErr);

} /* aStem_GetLibRef */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem_GetNamedLibRef
 */

aLIBRETURN 
aStem_GetNamedLibRef(aStemLib* pStemRef, 
		     const char* pName,
		     aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = NULL;

  if (!pStemRef)
    err = aErrParam;

  /* build the stem object */
  if (err == aErrNone) {
    pStem = (aStem*)aMemAlloc(sizeof(aStem));
    if (pStem) {
      aBZero(pStem, sizeof(aStem));
      pStem->check = aSTEMCHECK;
    } else {
      err = aErrMemory;
    }
  }

  /* stash the name */
  if (pName)
    aStringCopySafe(pStem->sName, aSTEMMAXNAMELENGTH, pName);
  
  /* open up the io library */
  if (err == aErrNone)
    aIO_GetLibRef(&pStem->ioRef, &err);
  
  /* now, set up the stem */
  if (err == aErrNone) {
    aPacket_Init(&pStem->curLinkPacket, aPacketFromStem);
    aPacket_Init(&pStem->curRelayPacket, 0);
    if (err == aErrNone)
      aMemPool_Create(pStem->ioRef, sizeof(aPacket), 
        	      aPACKETPOOLBLOCKSIZE, &pStem->packetPoolRef,
        	      &err);
    if (err == aErrNone)
      err = aPacketList_Create(pStem->packetPoolRef,
        		   	   &pStem->pLinkPacketList);
    if (err == aErrNone)
      err = aPacketList_Create(pStem->packetPoolRef,
        		   	   &pStem->pRelayPacketList);
    if (err == aErrNone)
      *((aStem**)pStemRef) = pStem;
    else
      sStem_Destroy(pStem);
  }

  /* get settings file */
  if (!pStem->stemSettings) {
    aSettingFile_Create(pStem->ioRef, 
			aSTEM_MAXSETTINGLEN,
			aSTEM_SETTINGSFILE,
			&pStem->stemSettings,
			&err);
    aAssert(err == aErrNone);
  }

  /* extract settings */
  if (err == aErrNone)
    aSettingFile_GetInt(pStem->ioRef, 
    			pStem->stemSettings, 
    			aSTEM_KEY_TEADELAY, 
    			&pStem->nTEADataDelayMS,
    			aSTEM_TEADELAY_DEFAULT,
    			&err);

  /* try to find and setup a debug file */
  if (err == aErrNone) {
    aErr ioErr;
    aFileRef testFile;
    char sNewName[aSTEMMAXNAMELENGTH*2];
    
    aAssert(pStem->ioRef);

    /* construct name of debug file */
    aStringCopySafe(sNewName, aSTEMMAXNAMELENGTH * 2, aSTEM_DEBUGFILENAME);
    if (aStringLen(pStem->sName))
      aStringCatSafe(sNewName, aSTEMMAXNAMELENGTH * 2, pStem->sName);

    /* check for debug file */
    aFile_Open(pStem->ioRef, sNewName, 
  	       aFileModeReadOnly, aFileAreaBinary,
  	       &testFile, &ioErr);

    /* just close it if you found it */
    if (ioErr == aErrNone)
      aFile_Close(pStem->ioRef, testFile, &ioErr);

    /* then open it for writing if it is there */
    if (ioErr == aErrNone)
      aStream_CreateFileOutput(pStem->ioRef, sNewName,
        		       aFileAreaBinary, &pStem->debugStream, 
      			       &ioErr);

    /* write a preamble */
    if (ioErr == aErrNone)
      aStream_WriteLine(pStem->ioRef, pStem->debugStream,
      			aSTEM_DBG_LIBOPEN, &ioErr);
  }

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStem_GetNamedLibRef */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem_ReleaseLibRef
 */

aLIBRETURN aStem_ReleaseLibRef(aStemLib stemRef, 
			       aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;
  aErr ioErr;
  int i;

  aVALIDSTEM(pStem);

  /* dump any relays that may be set up */
  for (i = 0; (err == aErrNone) && (i < aSTEMMAXRELAYS); i++) {
    if (pStem->relays[i])
      aStream_Destroy(pStem->ioRef, pStem->relays[i], &err);
  }

  /* close down the relay stream */
  if (err == aErrNone)
    aStem_SetStream(stemRef, NULL, kStemRelayStream, &ioErr);

  /* close down the module stream */
  if (err == aErrNone)
    aStem_SetStream(stemRef, NULL, kStemModuleStream, &ioErr);

  /* close down the settings file */
  if ((err == aErrNone)
      && (pStem->stemSettings != NULL)) {
    aSettingFile_Destroy(pStem->ioRef, 
			 pStem->stemSettings,
			 &err);
  }

  /* close up the debug file if it is open */
  if ((err == aErrNone)
      && (pStem->debugStream != NULL)) {
    aStream_WriteLine(pStem->ioRef, 
      		      pStem->debugStream,
      		      aSTEM_DBG_LIBCLOSE,
      		      &ioErr);
    aStream_Destroy(aStreamLibRef(pStem->debugStream), 
		    pStem->debugStream, &ioErr);
  }

  /* clean up the packet lists */
  if ((err == aErrNone) && (pStem->pRelayPacketList))
    err = aPacketList_Destroy(pStem, pStem->pRelayPacketList);
  if ((err == aErrNone) && (pStem->pLinkPacketList))
    err = aPacketList_Destroy(pStem, pStem->pLinkPacketList);

  if ((err == aErrNone) && (pStem->packetPoolRef))
    aMemPool_Destroy(pStem->ioRef, pStem->packetPoolRef, &err);

  /* clean up any reference we have to the io library */
  if ((err == aErrNone)
      && (pStem->ioRef != NULL))
    aIO_ReleaseLibRef(pStem->ioRef, &ioErr);

  if (err == aErrNone)
    aMemFree((aMemPtr)pStem);

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStem_ReleaseLibRef */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem_GetVersion
 */

aLIBRETURN 
aStem_GetVersion(aStemLib libRef,
		 unsigned long *pVersion,
		 aErr* pErr)
{
  aErr err = aErrNone;

  aVALIDSTEM(libRef);

  if ((err == aErrNone) && (pVersion == NULL))
    err = aErrParam;

  if (err == aErrNone)
    *pVersion = aVERSION_PACK(aVERSION_MAJOR, 
			      aVERSION_MINOR, 
			      aSTEM_BUILD_NUM); 

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStem_GetVersion */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem_SetStream
 */

aLIBRETURN 
aStem_SetStream(aStemLib stemRef, 
		aStreamRef streamRef,
		aStemStreamType streamType,
		aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;
  aStreamRef* pStream = NULL;

  aVALIDSTEM(pStem);

  /* get the stream in question */
  if (err == aErrNone) {

    switch (streamType) {

    case kStemModuleStream:
      pStream = &pStem->linkStream;
      if (pStem->debugStream) {
	aStream_WriteLine(aStreamLibRef(pStem->debugStream),
			  pStem->debugStream,
			  aSTEM_DBG_MODULESTREAM, NULL);
      }
      break;

    case kStemRelayStream:
      pStream = &pStem->relayStream;
      if (pStem->debugStream) {
	aStream_WriteLine(aStreamLibRef(pStem->debugStream),
			  pStem->debugStream,
			  aSTEM_DBG_RELAYSTREAM, NULL);
      }
      break;

    } /* switch */
  }

  if (pStream == NULL)
    err = aErrParam;

  /* throw away any old stream being used */
  if ((err == aErrNone) && (pStream != NULL) && (*pStream != NULL)) {
    aStreamRef stream = *pStream;
    *pStream = NULL;
    if (pStem->debugStream) {
      aStream_WriteLine(aStreamLibRef(pStem->debugStream),
      			pStem->debugStream,
      			aSTEM_DBG_STREAMCLEARED, NULL);
    }
    aStream_Destroy(aStreamLibRef(stream),
      		    stream, 
      		    &err);
    if ((err == aErrNone) && (pStem->debugStream)) {
      aStream_WriteLine(aStreamLibRef(pStem->debugStream),
      			pStem->debugStream,
      			aSTEM_DBG_STREAMDISPOSED, NULL);
    }
  }

  /* set the new stream */
  if ((err == aErrNone) && (streamRef != NULL)) {
    *pStream = streamRef;
    if (pStem->debugStream) {
      aStream_WriteLine(aStreamLibRef(pStem->debugStream),
      			pStem->debugStream,
      			aSTEM_DBG_STREAMSET, NULL);
    }
  }

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStem_SetStream */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem_SetHBCallback
 */

aLIBRETURN 
aStem_SetHBCallback(aStemLib stemRef, 
		    aHeartbeatCallback cbProc,
		    void* cbRef,
		    aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;

  aVALIDSTEM(pStem);

  if (err == aErrNone) {
    pStem->heartbeatCB = cbProc;
    pStem->heartbeatCBRef = cbRef;
  }

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStem_SetHBCallback */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem_DebugLine
 */

aLIBRETURN 
aStem_DebugLine(aStemLib stemRef,
		const char *line,
		aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;

  aVALIDSTEM(pStem);
  if (line == NULL)
    err = aErrParam;

  if ((err == aErrNone) && pStem->debugStream)
    aStream_WriteLine(aStreamLibRef(pStem->debugStream),
    		      pStem->debugStream, 
		      line, 
		      &err);

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStem_DebugLine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem_GetPacket
 *
 * Strategy:
 *
 * We need to follow a prescribed path to support all the cases.
 * First, we need to flush the stream of all bytes and accumulate 
 * these bytes into packets in the packet list.
 *
 * Now there are two options, filtered and not filtered.
 *
 *  Filtered: In this case, we walk the list applying the 
 *            filter to look for a match.  If the filter
 *            returns true, we stop looking and return 
 *	      the packet with one exception.  If the 
 *	      packet is a heartbeat packet, we service
 *	      the packet, don't give it back, and return 
 *	      an aErrNotFound message.
 *
 * Unfiltered: Here, the entire list is walked to service
 *	      (and remove) the heartbeat packets and we 
 *	      return the packet at the head of the list 
 *	      if it is present.
 *
 * We want to handle the following cases:
 *
 * 1. no filter, several packets arrive, we cleans the
 *    list of the heartbeats (to keep things running 
 *    smoothly) and return the first packet that was 
 *    available.
 * 2. no filter, only one packet that is a heartbeat
 *    we service the heartbeat and return aErrNotFound.
 * 3. filter, we apply the filter to find the first
 *    match but we also handle all heartbeat packets
 * 4. filter, none match, we still clean out the 
 *    hearbeats and return aErrNotFound
 *
 * Basically, we always process any available heartbeats
 * since we have the chance and the heartbeats never 
 * get returned.  If a filter is present, it sees 
 * every packet, including heartbeats.
 */

aLIBRETURN aStem_GetPacket(aStemLib stemRef, 
			   aPacketFilter filterProc,
			   void* filterRef,
			   unsigned long nMSTimeout,
			   aPacketRef* pPacketRef,
			   aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;
  aPacket* pPacket = NULL;
  unsigned long nStart, nCurrent;

  aVALIDSTEM(pStem);
  if (!pPacketRef)
    err = aErrParam;

  /* just say no if no stream has been set */
  if ((err == aErrNone) && (pStem->linkStream == NULL))
    err = aErrNotReady;

  /* establish the start time for timeouts */
  if (err == aErrNone)
    aIO_GetMSTicks(pStem->ioRef, &nStart, &err);

  /* if not relaying, try to filter and retrieve the packet */
  if (pStem->relayStream == NULL) {

    /* if a filter proc was set, continue to request packets
     * until the filter finds what it wants or a timeout 
     * occurs
     */
    find_packet:

    /* suck in any available packets into the packet list, regardless
     * of what type they are */
    if (err == aErrNone)
      err = aStem_DrainStream(pStem,
			      pStem->linkStream,
			      &pStem->curLinkPacket,
			      filterProc, filterRef,
			      aTrue,
			      aPacketFromStem,
			      pStem->pLinkPacketList);

    /* see if one is available in the list and return it */
    if (err == aErrNone) {
      err = aPacketList_GetFirst(pStem->pLinkPacketList, 
    				     filterProc, filterRef,
    				     &pPacket);
      if (err == aErrNone)
	*((aPacket**)pPacketRef) = pPacket;
    }

    /* if the timeout expired, stop trying */
    if (err == aErrNotFound) {
      aIO_GetMSTicks(pStem->ioRef, &nCurrent, &err);
      if (nCurrent - nStart >= nMSTimeout) {
        err = aErrTimeout;
      } else {
        err = aErrNone;
        aIO_MSSleep(pStem->ioRef, 1, NULL);
        goto find_packet;
      }
    }

  /* if a relay is set, just get any packets and relay */
  } else {

    /* drain both streams */
    if (err == aErrNone) {
      err = aStem_DrainStream(pStem, 
			      pStem->linkStream,
			      &pStem->curLinkPacket,
			      filterProc, filterRef,
			      aFalse,
			      aPacketFromStem,
			      pStem->pLinkPacketList);
    } 

    if (err == aErrNone) {
      err = aStem_DrainStream(pStem, 
			      pStem->relayStream,
			      &pStem->curRelayPacket,
			      filterProc, filterRef,
			      aFalse,
			      0,
			      pStem->pRelayPacketList);
      
    }

    /* check for module -> relay packets and send along */
    if (err == aErrNone) {
      err = aPacketList_GetFirst(pStem->pLinkPacketList, 
				 filterProc, filterRef,
				 &pPacket);   
      if (err == aErrNone) {
        err = aPacket_WriteToStream(pPacket, pStem->relayStream);
        aPacket_Destroy((aStemLib)pStem, pPacket, NULL);
      } else if (err == aErrNotFound) {
        err = aErrNone;
      }
    }

    /* check for relay -> module packets */
    if (err == aErrNone) {
      err = aPacketList_GetFirst(pStem->pRelayPacketList, 
				 filterProc, 
				 filterRef, 
				 &pPacket);
      if (err == aErrNone) {

        /* we let the heartbeat callback see the heartbeats on 
         * the way through */
        if (aPACKET_IS_HB(pPacket)) {
          if (pStem->heartbeatCB) {
            aBool bHBOn;
            bHBOn = ((pPacket->data[1] == 0) 
                     || (pPacket->data[1] == 0)) ? aTrue : aFalse;
            err = pStem->heartbeatCB(bHBOn, pStem->heartbeatCBRef);
          }
        }

        err = aPacket_WriteToStream(pPacket, pStem->linkStream);
        aPacket_Destroy((aStemLib)pStem, pPacket, NULL);
      } else if (err == aErrNotFound) {
        err = aErrNone;
      }
    }

    /* check to see if the relay was dropped and clear the relay */
    if (err != aErrIO)
      /* when relaying, we never get packets */
      err = aErrNotFound;
  }

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStem_GetPacket */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem_SendPacket
 */

aLIBRETURN 
aStem_SendPacket(aStemLib stemRef,
		 const aPacketRef packetRef,
		 aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;
  aPacket* pPacket = (aPacket*)packetRef;

  aVALIDSTEM(pStem);
  aVALIDPACKET(pPacket);

  /* make sure we have a data size */
  if ((err == aErrNone) && 
      (pPacket->dataSize > aSTEMMAXPACKETBYTES))
    err = aErrRange;

  /* make sure we have a stream */
  if ((err == aErrNone) && (pStem->linkStream == NULL))
    err = aErrIO;
  
  /* send the packet down the stream */
  if (err == aErrNone)
    err = aPacket_WriteToStream(pPacket, pStem->linkStream);

  /* output the debug packet if debug stream present */
  if (err == aErrNone)
    err = sStem_LogDebugPacket(pStem, pPacket);

  /* always clean up the packet */
  aMemPool_Free(pStem->ioRef, 
  		pStem->packetPoolRef, 
    		pPacket, 
    		NULL);

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStem_SendPacket */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem_CreateRelayStream
 */

aLIBRETURN 
aStem_CreateRelayStream(aStemLib stemRef,
			const unsigned char address,
			aStreamRef* pStreamRef,
			aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;
  int i;

  aVALIDSTEM(pStem);
  if ((address % 1) || (address == 0) || !pStreamRef)
    err = aErrParam;

  /* see if this is already set up as a relay */
  if (err == aErrNone) {
    i = aRELAYINDEX(address);
    if (pStem->relays[i])
      err = aErrBusy;
  }
  
  if (err == aErrNone)
    err = aSerialRelayStream_Create(pStem,
				    address,
				    pStreamRef,
				    &pStem->relays[i]);

  if (pErr != NULL)
    *pErr = err;
  
  return (aLIBRETURN)(err != aErrNone);

} /* aStem_CreateRelayStream */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStem_LogDebugPacket
 */

aErr 
sStem_LogDebugPacket(aStem* pStem,
		     aPacket* pPacket)
{
  aErr err = aErrNone;

  /* add the packet to the debug stream if present */
  if (pStem->debugStream) {
    aErr dbgErr;
    char debug[aSTEMMAXDEBUGLINE];
    aPacket_Format((aStemLib)pStem,
		   (aPacketRef)pPacket, 
		   debug, 
		   aSTEMMAXDEBUGLINE, 
		   &dbgErr);
    if (dbgErr == aErrNone)
      aStream_WriteLine(aStreamLibRef(pStem->debugStream),
                	pStem->debugStream, debug, NULL);
  }

  return err;

} /* sStem_LogDebugPacket */



#if RAWDEBUG
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRAWDEBUG
 *
 * Shows all raw bytes coming in on link stream.
 */

void sRAWDEBUG(aStem* pStem, const char byte)
{
  char buf[5];
  char* p = buf;
  char hexlookup[16] = {'0', '1', '2', '3', 
  			'4', '5', '6', '7',
			'8', '9', 'A', 'B', 
			'C', 'D', 'E', 'F'};
  *p++ = 'r';
  *p++ = ' ';
  *p++ = hexlookup[(byte >> 4) & 0x0F];
  *p++ = hexlookup[byte & 0x0F];
  *p = 0;
  aStem_DebugLine((aStemLib)pStem, buf, NULL);
}
#endif /* sRAWDEBUG */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem_DrainStream
 *
 * This function drains out the input stream of any inbound bytes
 * and then returns.  There is no error if there are no packets 
 * available.
 */

aErr 
aStem_DrainStream(aStem* pStem,
		  aStreamRef stream,
		  aPacket* pPacketStore,
		  aPacketFilter filterProc,
		  void* filterRef,
		  aBool bHandleBackChannel,
		  unsigned char initflags,
		  aPacketList* pPacketList)
{
  aErr stemErr = aErrNone;
  aErr readErr = aErrNone;
  char byte;

  aAssert(pStem);

  if ((stemErr == aErrNone) && (stream != NULL)) {

    /* we are out of sync so try to re-establish sync */
    if (pStem->nReSyncState) {

      switch (pStem->nReSyncState) {

      /* in this state, we are out of sync and starting to
       * re-sync the packet framing */      
      case kSyncStateStart: {
        unsigned long now, quit;
        unsigned char magic[4] = {cmdMAGIC, 0, cmdMAGIC, 0};

	aStem_DebugLine((aStemLib)pStem, aSTEM_DBG_SYNCFLUSHING, NULL);

        /* first, flush out any inbound bytes and make sure we
         * don't get stuck here forever */
        aIO_GetMSTicks(pStem->ioRef, &now, &stemErr);
        if (stemErr == aErrNone) {
          quit = now + 250; /* 1/4 second */
          do {
            aStream_Read(aStreamLibRef(stream), 
      		         stream, &byte, 1, &readErr);
#if RAWDEBUG
	    if (readErr == aErrNone)
	      sRAWDEBUG(pStem, byte);
#endif /* RAWDEBUG */
            aIO_GetMSTicks(pStem->ioRef, &now, &stemErr);
          } while ((readErr == aErrNone) 
        	   && (stemErr == aErrNone)
        	   && (now < quit));
       
          /* if we got some other error, show it */
          if ((readErr != aErrNone) && (readErr != aErrNotReady))
            stemErr = readErr;

          /* if we ran out of time, show not ready */
          if ((stemErr == aErrNone) && (now >= quit))
            stemErr = aErrNotFound;
        }

        /* now, send down the magic sequence bytes */
        if (stemErr == aErrNone) {
	  aStem_DebugLine((aStemLib)pStem, aSTEM_DBG_SYNCPHASE1, NULL);
          aStream_Write(aStreamLibRef(stream), stream,
      		        (char*)magic, 4, &stemErr);
        }

        /* if we made it here, advance the sync state */
        if (stemErr == aErrNone) {
          pStem->nReSyncState = kSyncStateByte1;
	  aStem_DebugLine((aStemLib)pStem, aSTEM_DBG_SYNCPHASE2, NULL);
	}
      }
      break;

      /* In this state, we have flushed the inbound serial and 
       * sent magic bytes.  We now look for the next byte as 
       * a potential address byte. */
      case kSyncStateByte1: {
        /* try to get a byte */
        aStream_Read(aStreamLibRef(stream), 
      		     stream, &byte, 1, &readErr);
        if (readErr == aErrNone) {

	  sRAWDEBUG(pStem, byte);

          /* if it was not a valid address, restart */
          if ((byte == 0) || (byte % 2)) {
            pStem->nReSyncState = kSyncStateStart;
          }

          /* otherwise, hold onto it */
          else {
            pStem->cSyncCache = byte;
            pStem->nReSyncState = kSyncStateByte2;
	    aStem_DebugLine((aStemLib)pStem, aSTEM_DBG_SYNCPHASE3, NULL);
          }
        } else if (readErr != aErrNotReady)
          stemErr = readErr;
      }
      break;

      /* In this state, we have a potential address byte.  
       * The next byte should be a zero. */
      case kSyncStateByte2: 
      case kSyncStateByte4: {

        /* try to get a byte */
        aStream_Read(aStreamLibRef(stream), 
      		     stream, &byte, 1, &readErr);
        if (readErr == aErrNone) {

	  sRAWDEBUG(pStem, byte);

          /* if it was not a zero, restart */
          if (byte) {
            pStem->nReSyncState = kSyncStateStart;
          }

          /* otherwise, see if we are done */
          else if (pStem->nReSyncState == kSyncStateByte4) {
            pStem->nReSyncState = kSyncStateSynced;
	    aStem_DebugLine((aStemLib)pStem, aSTEM_DBG_SYNCAQUIRED, NULL);

          } else {
            pStem->nReSyncState = kSyncStateByte3;
	    aStem_DebugLine((aStemLib)pStem, aSTEM_DBG_SYNCPHASE4, NULL);
	  }

        } else if (readErr != aErrNotReady)
          stemErr = readErr;
      }
      break;

      /* In this state, we have a potential address byte.  
       * The next byte should be a zero. */
      case kSyncStateByte3: {
        /* try to get a byte */
        aStream_Read(aStreamLibRef(stream), 
      		     stream, &byte, 1, &readErr);
        if (readErr == aErrNone) {

	  sRAWDEBUG(pStem, byte);

          /* if it was not the same address, restart */
          if (byte != pStem->cSyncCache) {
	    aStem_DebugLine((aStemLib)pStem, "restart 3", NULL);
            pStem->nReSyncState = kSyncStateStart;
          }
          
          /* otherwise, advance the state */
          else
            pStem->nReSyncState = kSyncStateByte4;

        } else if (readErr != aErrNotReady)
          stemErr = readErr;
      }
      break;

      } /* switch */

      /* always report not ready (as though no bytes available) */
      if (stemErr == aErrNone)
        stemErr = aErrNotFound;


    } else {

      // we are in packet sync
      do {
        if (!aStream_Read(aStreamLibRef(stream), 
      		          stream, &byte, 1, &readErr)) {

	  sRAWDEBUG(pStem, byte);

          stemErr = aPacket_AddByte(pPacketStore, 
        			    (unsigned char)byte);

          if ((stemErr == aErrNone) && aPacket_Complete(pPacketStore)) {

	    /* record reciept of the packet */
            sStem_LogDebugPacket(pStem, pPacketStore);

	    /* see if it is a heartbeat packet we should handle */
            if (stemErr == aErrNone) {
              aErr hbErr;
              if (bHandleBackChannel == aTrue)
                hbErr = sStem_HandleBackChannel(pStem, 
						pPacketStore,
						filterProc, 
						filterRef,
						pStem->pLinkPacketList);
      	      else
      	        hbErr = aErrNotFound;

              /* all non-hb packets get added to the list */
      	      if (hbErr == aErrNotFound) {
                stemErr = aPacketList_AddCopy(pStem, 
              				      pPacketList, 
              				      pPacketStore);

              /* show an error if we didn't handle the hb */
              } else if (hbErr != aErrNone) {
                stemErr = hbErr;
	      }
            }

            /* clean up and start with a fresh packet */
            aPacket_Init(pPacketStore, initflags);
          }

	  /* if we got a packet error, we dropped out of packet
	   * synchronization */
          if (stemErr == aErrPacket) {
            pStem->nReSyncState = 1;
            pStem->cSyncCache = kSyncStateStart;

            /* toss existing packet */
            aPacket_Init(pPacketStore, initflags);

	    /* log the sync error */
	    aStem_DebugLine((aStemLib)pStem, aSTEM_DBG_SYNCLOST, NULL);

	    /* show we don't have a packet */
	    stemErr = aErrNotFound;
          }
        }
      } while ((stemErr == aErrNone) && (readErr == aErrNone));
    } /* if in sync */
  } /* if stream and no errors */

  return(stemErr);

} /* aStem_DrainStream */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStem_HandleBackChannel
 *
 * checks the packet and responds to it if it is a heartbeat or
 * other back channel packet.
 * aErrNone means it was back channel, aErrNotFound means it wasn't.
 * other errors where appropriate
 */

aErr 
sStem_HandleBackChannel(aStem* pStem,
			aPacket* pPacket,
			aPacketFilter filterProc,
			void* filterRef,
			aPacketList* pPacketList)
{
  aErr stemErr = aErrNone;

  /* automatically respond to heatbeat packets here */
  if (aPACKET_IS_HB(pPacket)) {
    aPacket* pCopy;

    /* let the heartbeat CB see the see the heartbeat that has
     * come through */
    if (pStem->heartbeatCB) {
      aBool bHBOn;
      bHBOn = ((pPacket->data[1] == 0) || (pPacket->data[1] == 0)) ? 
      	      aTrue : aFalse;
      stemErr = pStem->heartbeatCB(bHBOn, pStem->heartbeatCBRef);
    }

    if (pStem->relayStream == NULL) {

      /* build a copy that will be disposed when sent */
      void* p;
      aMemPool_Alloc(pStem->ioRef, pPacketList->packetPoolRef, &p, &stemErr);

      if (stemErr == aErrNone) {
        pCopy = (aPacket*)p;
        *pCopy = *pPacket;

        /* just send back the same packet modified to be a reply */
        pCopy->data[1] += 2;
        pCopy->status &= (unsigned char)~aPacketFromStem;
        aStem_SendPacket((aStemLib)pStem, pCopy, &stemErr);
      }
    } else {
      stemErr = aErrNotFound;
    }
  
  /* buffer payload data from serial relay channel packets */
  } else if (aPACKET_IS_RELAY(pPacket)) {
    /* compute the stream index for the module that 
     * sent the packet */
    int i = aRELAYINDEX(pPacket->address);
    if (pStem->relays[i]) {
      aStream_Write(pStem->ioRef, 
      		    pStem->relays[i]->buffer,
      		    &pPacket->data[1],
      		    (aMemSize)(pPacket->dataSize - 1),
      		    &stemErr);
    }

  /* otherwise, just return that it was not a backchannel packet */
  } else {
    stemErr = aErrNotFound;
  }

  return stemErr;

} /* sStem_HandleBackChannel */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStem_Destroy
 */

static aErr
sStem_Destroy(aStem* pStem)
{
  aErr err = aErrNone;
  aErr ioErr;
  int i;

  aVALIDSTEM(pStem);

  /* dump any relays that may be set up */
  for (i = 0; (err == aErrNone) && (i < aSTEMMAXRELAYS); i++) {
    if (pStem->relays[i])
      aStream_Destroy(pStem->ioRef, pStem->relays[i], &err);
    aAssert(!pStem->relays[i]);
  }

  /* close down the relay stream */
  if (err == aErrNone)
    aStem_SetStream((aStemLib)pStem, NULL, kStemRelayStream, &ioErr);

  /* close down the module stream */
  if (err == aErrNone)
    aStem_SetStream((aStemLib)pStem, NULL, kStemModuleStream, &ioErr);

  /* close down the settings file */
  if ((err == aErrNone) && (pStem->stemSettings != NULL)) {
    aSettingFile_Destroy(pStem->ioRef, 
			 pStem->stemSettings,
			 &err);
  }

  /* close up the debug file if it is open */
  if ((err == aErrNone) && (pStem->debugStream != NULL)) {
    aStream_WriteLine(pStem->ioRef, 
		      pStem->debugStream,
		      aSTEM_DBG_LIBCLOSE,
		      &ioErr);
    aStream_Destroy(aStreamLibRef(pStem->debugStream), 
		    pStem->debugStream, &ioErr);
  }

  /* clean up the packet lists */
  if ((err == aErrNone) && (pStem->pRelayPacketList))
    err = aPacketList_Destroy(pStem, pStem->pRelayPacketList);
  if ((err == aErrNone) && (pStem->pLinkPacketList))
    err = aPacketList_Destroy(pStem, pStem->pLinkPacketList);

  if ((err == aErrNone) && (pStem->packetPoolRef))
    aMemPool_Destroy(pStem->ioRef, pStem->packetPoolRef, &err);

  /* clean up any reference we have to the io library */
  if ((err == aErrNone)
      && (pStem->ioRef != NULL))
    aIO_ReleaseLibRef(pStem->ioRef, &ioErr);

  if (err == aErrNone)
    aMemFree((aMemPtr)pStem);

  return err;

} /* sStem_Destroy */
