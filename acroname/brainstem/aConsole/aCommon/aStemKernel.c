/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStemKernel.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent BrainStem */
/*		console object.					   */
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


#include "aStemKernel.h"

#define	aSKTASKLISTSIZE		8


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemKernel_Create
 */

aErr aStemKernel_Create(aIOLib ioRef,
			aStemKernel** ppKernel)
{
  aErr stemErr = aErrNone;
  aStemKernel* pKernel;
  
  aAssert(ppKernel);

  pKernel = (aStemKernel*)aMemAlloc(sizeof(aStemKernel));
  if (pKernel == NULL)
    stemErr = aErrMemory;
  
  if (stemErr == aErrNone) {
    aBZero(pKernel, sizeof(aStemKernel));
    aMemPool_Create(ioRef, sizeof(aStemTask), 
    		    aSKTASKLISTSIZE, 
    		    &pKernel->taskPool, &stemErr);
    if (stemErr != aErrNone)
      aStemKernel_Destroy(pKernel);
    else {
      pKernel->ioRef = ioRef;
      *ppKernel = pKernel;
    }
  }
  
  return stemErr;

} /* aStemKernel_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemKernel_AddPacket
 */

aErr aStemKernel_AddPacket(aStemKernel* pKernel,
			   aPacketRef packet)
{
  aErr stemErr = aErrNone;
  void* pMemory;
  aStemTask* pTask;

  if (pKernel == NULL) {
    stemErr = aErrParam;
  } else {
    /* build a new list item for it */
    aMemPool_Alloc(pKernel->ioRef, pKernel->taskPool, &pMemory, &stemErr); 
  }

  /* insert it at the end of the list */
  if (stemErr == aErrNone) {
    pTask = (aStemTask*)pMemory;
    pTask->pNext = NULL;
    if (pKernel->pHead == NULL) {
      pKernel->pHead = pTask;
    } else {
      pKernel->pTail->pNext = pTask;
    }
    pKernel->pTail = pTask;
    pTask->packet = packet;
  }

  return stemErr;

} /* aStemKernel_AddPacket */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemKernel_PacketAvailable
 */

aBool aStemKernel_PacketAvailable(aStemKernel* pKernel,
			          aPacketRef* pPacket)
{
  aErr stemErr = aErrNone;

  aAssert(pKernel);
  aAssert(pPacket);

  if (pKernel->pHead) {
    aStemTask* pTask = pKernel->pHead;
    pKernel->pHead = pTask->pNext;
    *pPacket = pTask->packet;
    aMemPool_Free(pKernel->ioRef, pKernel->taskPool, 
    		  pTask, &stemErr);
    return (stemErr == aErrNone);
  }
  
  return aFalse;

} /* aStemKernel_PacketAvailable */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemKernel_Create
 */

aErr aStemKernel_Destroy(aStemKernel* pKernel)
{
  aErr stemErr = aErrNone;
  
  if (pKernel) {
    if (pKernel->taskPool) {
      while (pKernel->pHead && (stemErr == aErrNone)) {
        aStemTask* pTask = pKernel->pHead;
        pKernel->pHead = pTask->pNext;
        aMemPool_Free(pKernel->ioRef, pKernel->taskPool, 
        	      pTask, &stemErr);
      }
      if (stemErr == aErrNone)
        aMemPool_Destroy(pKernel->ioRef, pKernel->taskPool, NULL);
    }
    aMemFree((aMemPtr)pKernel);
  }

  return stemErr;

} /* aStemKernel_Destroy */
