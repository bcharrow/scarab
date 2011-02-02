/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aPacket.c                                                 */
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

#include "aStemInternal.h"
#include "aPacket.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacket_Init
 */

void 
aPacket_Init(aPacket* pPacket,
	     unsigned char initflags)
{
  if (!pPacket)
    return;

  aBZero(pPacket, sizeof(aPacket));
  pPacket->check = aPACKETCHECK;
  pPacket->status = initflags; 

} /* end of aPacket_Init */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacket_Size
 */

int 
aPacket_Size(aPacket* pPacket)
{
  if (!pPacket)
    return 0;
  
  return(pPacket->dataSize);
  
} /* end of aPacket_Size */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacket_BytesSoFar
 */

int 
aPacket_BytesSoFar(aPacket* pPacket)
{
  if (!pPacket)
    return 0;

  return(pPacket->curSize);

} /* end of aPacket_BytesSoFar routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacket_Complete
 */

aBool 
aPacket_Complete(aPacket* pPacket)
{
  if (!pPacket)
    return aFalse;

  if (!(pPacket->status & aPacketHasSize))
    return aFalse;

  if (pPacket->curSize == pPacket->dataSize + 2)
    return aTrue;

  return aFalse;
  
} /* end of aPacket_Complete routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacket_AddByte
 */

aErr 
aPacket_AddByte(aPacket* pPacket, 
		unsigned char byte)
{
  if (!pPacket)
    return aErrParam;
  
  pPacket->curSize++;
  
  /* account for cmd and address */
  if (pPacket->curSize > aSTEMMAXPACKETBYTES + 2)
    return aErrUnknown;
  
  if (!(pPacket->status & aPacketHasAddr)) {
    
    /* check for invalid address */
    if ((byte == 0) || (byte % 2))
      return aErrPacket;
    
    pPacket->status |= aPacketHasAddr;
    pPacket->address = byte;
    return aErrNone;
  }
  
  if (!(pPacket->status & aPacketHasSize)) {
    pPacket->status |= aPacketHasSize;
    pPacket->dataSize = byte;
    
    /* check for invalid size byte */
    if (pPacket->dataSize > aSTEMMAXPACKETBYTES)
      return aErrPacket;
    
    return aErrNone;
  }

  pPacket->data[pPacket->curSize - 3] = (char)byte;
  
  return aErrNone;
  
} /* end of aPacket_AddByte routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacket_WriteToStream
 */
 
aErr 
aPacket_WriteToStream(aPacket* pPacket,
		      aStreamRef stream)
{
  aErr err = aErrNone;
  unsigned char buffer[aSTEMMAXPACKETBYTES + 2]; /* data + length + address */

  /* accumulate the data into a single buffer */
  buffer[0] = pPacket->address;  /* address */ 
  buffer[1] = pPacket->dataSize; /* length */
  aMemCopy(&buffer[2], pPacket->data, pPacket->dataSize); /* data */

  /* dump to the stream */ 
  if (err == aErrNone)
    aStream_Write(aStreamLibRef(stream),
  		  stream,
  		  (char*)buffer,
  		  (unsigned long)(pPacket->dataSize + 2), 
  		  &err);

  return err;
 
} /* aPacket_WriteToStream */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacketList_Create
 */

aErr 
aPacketList_Create(aMemPoolRef packetPoolRef,
		   aPacketList** ppPacketList)
{
  aErr plErr = aErrNone;
  aPacketList* pPacketList;

  aAssert(packetPoolRef);
  aAssert(ppPacketList);

  if (plErr == aErrNone) {
    pPacketList = (aPacketList*)aMemAlloc(sizeof(aPacketList));
    if (pPacketList == NULL) {
      plErr = aErrMemory;
    } else {
      aBZero(pPacketList, sizeof(aPacketList));
      pPacketList->packetPoolRef = packetPoolRef;
      *ppPacketList = pPacketList;
    }
  }

  return plErr;

} /* aPacketList_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacketList_AddCopy
 *
 * adds a packet to the tail of the list
 */

aErr 
aPacketList_AddCopy(void* vpStem,
		    aPacketList* pPacketList,
		    aPacket* pPacket)
{
  aStem* pStem = (aStem*)vpStem;
  aErr err = aErrNone;
  aPacket* pCopy;

  aAssert(pPacketList);

  /* create the copy */
  if (err == aErrNone)
    aMemPool_Alloc(pStem->ioRef, pPacketList->packetPoolRef, 
		   (void*)&pCopy, &err);
 
  /* copy it from the passed-in Packet */
  if (err == aErrNone) 
    aMemCopy(pCopy, pPacket, sizeof(aPacket));

  /* insert it at the end of the list */
  if (err == aErrNone) {
    pPacketList->nSize++;
    pCopy->pNext = NULL;
    if (pPacketList->pHead == NULL) {
      pPacketList->pHead = pCopy;
    } else {
      pPacketList->pTail->pNext = pCopy;
    }
    pPacketList->pTail = pCopy;
  }

  return err;  

} /* aPacketList_AddCopy */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacketList_GetFirst
 * 
 * Starts looking at the front of the list for a packet.  This 
 * yields the oldest packet first. If a filter is passed in, it 
 * is used, otherwise, the first available packet is returned.
 */

aErr 
aPacketList_GetFirst(aPacketList* pPacketList,
		     aPacketFilter filterProc,
		     void* filterRef,
		     aPacket** ppPacket)
{
  aErr err = aErrNone;

  aAssert(pPacketList);
  aAssert(ppPacket);

  if (filterProc == NULL) {
  
    /* with no filter proc, just run down the list and look for 
     * a list head to find the first packet
     */
    if (pPacketList->pHead == NULL)
      err = aErrNotFound;
    else {
      pPacketList->nSize--;
      *ppPacket = pPacketList->pHead;
      pPacketList->pHead = pPacketList->pHead->pNext;
    }
  } else {

    /* with a filter proc, search the list to try to find 
     * the first packet the filter returns true on.
     */
    aPacket* pPacket = pPacketList->pHead;
    aPacket* pPrev = NULL;
    while ((pPacket != NULL) 
           && (filterProc(pPacket->address,
    			  pPacket->dataSize,
    			  pPacket->data,
    			  filterRef) == aFalse)
    	   && (!aPACKET_IS_HB(pPacket))) {
      pPrev = pPacket;
      pPacket = pPacket->pNext;
    }
    if (pPacket == NULL)
      err = aErrNotFound;
    else {
      /* record the find and remove it from the list */
      pPacketList->nSize--;
      *ppPacket = pPacket;
      if (pPrev == NULL)
        pPacketList->pHead = pPacket->pNext;
      else {
        pPrev->pNext = pPacket->pNext;
        if (pPacketList->pTail == pPacket)
          pPacketList->pTail = pPrev;
      }
    }
  }

  return err;

} /* aPacketList_GetFirst */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacketList_Destroy
 */

aErr 
aPacketList_Destroy(void* vpStem,
		    aPacketList* pPacketList)
{
  aStem* pStem = (aStem*)vpStem;
  aErr err = aErrNone;
  aPacket* pPacket;
  
  aAssert(pPacketList);
  aAssert(pPacketList->packetPoolRef);
  
  /* remove all the list elements */
  while((err == aErrNone) && (pPacketList->pHead != NULL)) {
    pPacket = pPacketList->pHead;
    pPacketList->pHead = pPacket->pNext;
    aMemPool_Free(pStem->ioRef, pPacketList->packetPoolRef, 
    		  pPacket, &err);
  }
  
  /* blow away the list itself */
  if (err == aErrNone)
    aMemFree((aMemPtr)pPacketList);

  return err;

} /* aPacketList_Destroy */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacket_Create
 */

aLIBRETURN
aPacket_Create(aStemLib stemRef,
	       const unsigned char address,
	       const unsigned char nLength,
	       const char* data,
	       aPacketRef* pPacketRef,
	       aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;

  aVALIDSTEM(stemRef);
  if ((data == NULL) || !pPacketRef)
    err = aErrParam;

  /* packet is limited in size */
  if ((err == aErrNone) && (nLength > aSTEMMAXPACKETBYTES))
    err = aErrSize;

  /* address can only be even */
  if ((err == aErrNone) && 
      ((address & 0x01) && (address != aSTEM_MAGICADDR)))
    err = aErrParam;

  /* build the packet */
  if (err == aErrNone)
    aMemPool_Alloc(pStem->ioRef, pStem->packetPoolRef, 
    		   (void*)pPacketRef, &err);
  
  if (err == aErrNone) {
    aPacket* pPacket = *((aPacket**)pPacketRef);
    aPacket_Init(pPacket, 0);
    pPacket->address = address;
    pPacket->dataSize = nLength;
    aMemCopy(pPacket->data, data, nLength);
  }

  if (pErr)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* end of aPacket_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacket_Format
 */

aLIBRETURN
aPacket_Format(aStemLib stemRef,
	       const aPacketRef packetRef,
	       char* pBuffer,
	       const unsigned short nMaxLength,
	       aErr* pErr)
{
  aErr err = aErrNone;
  aPacket* pPacket = (aPacket*)packetRef;
  char* p = pBuffer;
  unsigned short len = 0;
  int i;
  short  visLen = (short)(nMaxLength - 1);
  char hexlookup[16] = {'0', '1', '2', '3', 
			'4', '5', '6', '7',
			'8', '9', 'A', 'B', 
			'C', 'D', 'E', 'F'};

  aVALIDSTEM(stemRef);
  aVALIDPACKET(pPacket);

  if (err == aErrNone) {
    if (len++ >= visLen) goto end;
    *p++ = (char)((pPacket->status & aPacketFromStem) ? '<' : '>');
  
    if (len++ >= visLen) goto end;
    *p++ = ' ';

    if (len++ >= visLen) goto end;
    *p++ = hexlookup[(pPacket->address >> 4) & 0x0F];

    if (len++ >= visLen) goto end;
    *p++ = hexlookup[pPacket->address & 0x0F];

    if (len++ >= visLen) goto end;
    *p++ = ':';

    for (i = 0; (len++ < visLen) && (i < pPacket->dataSize); i++) {
      *p++ = hexlookup[(pPacket->data[i] >> 4) & 0x0F];
      if (len++ >= visLen) goto end;
      *p++ = hexlookup[pPacket->data[i] & 0x0F];
      if (i < pPacket->dataSize - 1) {
        if (len++ >= visLen) goto end;
        *p++ = ',';
      }
    }
  
  end:
    *p = 0; /* null terminate */
  }

  if (pErr)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* end of aPacket_Format routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacket_GetData
 */

aLIBRETURN
aPacket_GetData(aStemLib stemRef,
		const aPacketRef packetRef,
		unsigned char* pAddress,
		unsigned char* pLen,
		char* data,
		aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;
  aPacket* pPacket = (aPacket*)packetRef;

  aVALIDSTEM(pStem);
  aVALIDPACKET(pPacket);
  if ((data == NULL) || (pLen == NULL) || (pAddress == NULL))
    err = aErrParam;

  if (err == aErrNone) {
    aAssert(pPacket->dataSize <= aSTEMMAXPACKETBYTES);
    aAssert(!(pPacket->address & 0x01));
    *pAddress = pPacket->address;
    *pLen = pPacket->dataSize;
    aMemCopy(data, pPacket->data, pPacket->dataSize);
  }
  
  return (aLIBRETURN)(err != aErrNone);

} /* aPacket_GetData */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPacket_Destroy
 */

aLIBRETURN
aPacket_Destroy(aStemLib stemRef,
		aPacketRef packetRef,
		aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;
  aPacket* pPacket = (aPacket*)packetRef;

  aVALIDSTEM(pStem);
  aVALIDPACKET(pPacket);

  if (err == aErrNone) {
    aAssert(pStem->packetPoolRef);

    /* invalidate the packet */
    pPacket->check = 0;
    aMemPool_Free(pStem->ioRef, pStem->packetPoolRef, 
    		  pPacket, &err);
  }

  if (pErr)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aPacket_Destroy */


