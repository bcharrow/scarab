/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSerialRelayStream.c                                      */
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

#include "aIO.h"
#include "aStream.h"
#include "aSerialRelayStream.h"


#define aMAXPAYLOAD (aSTEMMAXPACKETBYTES - 1)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sSRSGet (
  char* pData,
  void* ref
);

static aErr sSRSPut (
  char* pData,
  void* ref
);

static aErr sSRSWrite(
  const char* pData,
  const unsigned long nSize, 
  void* ref
);

static aErr sSRSDelete(
  void* ref
);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSRSGet
 */

aErr sSRSGet (
  char* pData,
  void* ref
)
{
  aErr srsErr = aErrNone;
  aSerialRelayStream* pSRS = (aSerialRelayStream*)ref;

  /* drain the streams */
  if (srsErr == aErrNone) {
    aStem* pStem = pSRS->pStem;
    srsErr = aStem_DrainStream(pStem,
			       pStem->linkStream,
			       &pStem->curLinkPacket,
			       NULL, NULL,
			       aTrue,
			       aPacketFromStem,
			       pStem->pLinkPacketList);
  }
    			      

  /* now, just return anything in the buffer */
  if (srsErr == aErrNone)
    aStream_Read(pSRS->pStem->ioRef, pSRS->buffer, 
    		 pData, 1, &srsErr);
  
  /* change this error to be consistent with serial streams */
  if (srsErr == aErrEOF)
    srsErr = aErrNotReady;

  return srsErr;

} /* sSRSGet */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSRSPut
 */

aErr 
sSRSPut(char* pData,
	void* ref)
{
  aErr srsErr = aErrNone;
  aSerialRelayStream* pSRS = (aSerialRelayStream*)ref;
  char data[2];
  aPacket* pPacket = NULL;

  /* drain the streams */
  if (srsErr == aErrNone) {
    aStem* pStem = pSRS->pStem;
    srsErr = aStem_DrainStream(pStem,
			       pStem->linkStream,
			       &pStem->curLinkPacket,
			       NULL, NULL,
			       aTrue,
			       aPacketFromStem,
			       pStem->pLinkPacketList);
  }

  /* copy over the data */
  data[0] = cmdSER_RELAY;
  data[1] = *pData;

  if (srsErr == aErrNone) {
    aPacketRef pr;
    if (!aPacket_Create((aStemLib)pSRS->pStem, 
			pSRS->nAddress, 
			2, 
			data,
			&pr, 
			&srsErr))
      pPacket = (aPacket*)pr;
  }

  if (srsErr == aErrNone)
    aStem_SendPacket((aStemLib)pSRS->pStem, pPacket, &srsErr);

  /* build a single-byte serial relay packet and ship it */
  return srsErr;

} /* sSRSPut */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSRSWrite
 */

aErr sSRSWrite (
  const char* pData,
  const unsigned long nSize, 
  void* ref
)
{
  aErr srsErr = aErrNone;
  aSerialRelayStream* pSRS = (aSerialRelayStream*)ref;
  unsigned long nLeft = nSize;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packetRef;
  const char* p = pData;

  /* drain the streams */
  if (srsErr == aErrNone) {
    aStem* pStem = pSRS->pStem;
    srsErr = aStem_DrainStream(pStem,
			       pStem->linkStream,
			       &pStem->curLinkPacket,
			       NULL, NULL,
			       aTrue,
			       aPacketFromStem,
			       pStem->pLinkPacketList);
  }
  
  /* whitle down the input with serial relay packets stuffed full */
  /* packet looks like:
   * address
   * length
   * cmdSER_RELAY
   * data (1 - 7 bytes)
   */
  data[0] = cmdSER_RELAY;
  while ((srsErr == aErrNone) && (nLeft > 0)) {
    unsigned char nPacket;
    if (nLeft < aMAXPAYLOAD)
      nPacket = (unsigned char)nLeft;
    else
      nPacket = aMAXPAYLOAD;

    /* move in the memory */
    aMemCopy(&data[1], p, nPacket);
    p += nPacket;

    aPacket_Create((aStemLib)pSRS->pStem, 
		   pSRS->nAddress, 
		   (unsigned char)(nPacket + 1), 
		   data,
		   &packetRef,
		   &srsErr);

    if (srsErr == aErrNone)
      aStem_SendPacket((aStemLib)pSRS->pStem, packetRef, &srsErr);

    nLeft -= nPacket;
  }

  return srsErr;

} /* sSRSWrite */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSRSDelete
 */

aErr sSRSDelete (
  void* ref
)
{
  aErr srsErr = aErrNone;
  aSerialRelayStream* pSRS = (aSerialRelayStream*)ref;
  int i;

  /* yank this from the list of streams in the stem object */
  i = aRELAYINDEX(pSRS->nAddress);
  aAssert(pSRS->pStem->relays[i] == pSRS);
  pSRS->pStem->relays[i] = NULL;

  if ((srsErr == aErrNone) && pSRS->buffer) {
    aStream_Destroy(pSRS->pStem->ioRef, pSRS->buffer, &srsErr);
    pSRS->buffer = NULL;
  }

  if (srsErr == aErrNone)
    aMemFree(pSRS);

  return srsErr;

} /* sSRSDelete */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSerialRelayStream_Create
 */

aErr aSerialRelayStream_Create (
  aStem* pStem, 
  const unsigned char nAddress,
  aStreamRef* pStreamRef,
  aSerialRelayStream** ppRelayStream
)
{
  aErr srsErr = aErrNone;
  aSerialRelayStream* pSRS;

  if (srsErr == aErrNone) {
    pSRS = (aSerialRelayStream*)aMemAlloc(sizeof(aSerialRelayStream));
    if (pSRS)
      aBZero(pSRS, sizeof(aSerialRelayStream));
    else
      srsErr = aErrMemory;
  }
  
  if (srsErr == aErrNone) {
    pSRS->pStem = pStem;
    pSRS->nAddress = nAddress;
  }

  /* create the input buffer */
  if (srsErr == aErrNone)
    aStreamBuffer_Create(pSRS->pStem->ioRef, 32, 
    			 &pSRS->buffer, &srsErr);

  if (srsErr == aErrNone)
    aStream_Create(pSRS->pStem->ioRef,
    		   sSRSGet,
    		   sSRSPut,
    		   sSRSDelete,
    		   pSRS,
    		   pStreamRef,
    		   &srsErr);
  
  /* this is a bit of a hack (we peek into the private aIO structure
   * for streams) to handle block writes most efficiently.  Don't
   * try this at home.
   */
  if (srsErr == aErrNone) {
    aStream* pTemp = (aStream*)*pStreamRef;
    pTemp->writeProc = sSRSWrite;
    *ppRelayStream = pSRS;
  }

  return srsErr;
 
} /* aSerialRelayStream_Create */
