/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aMemPoolInternal.c					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: implementation of an efficient memory allocator    */
/*              for identical sized blocks. 	                   */
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
#include "aMemPoolInternal.h"

#ifdef aDEBUG
#define FREE_MEM_VAL  0xBB
#endif /* aDEBUG */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sAddBlock(aMemPool* pMP);
static aErr sAddToList(aMemPoolObject** ppList, 
		       aMemPoolObject* pObj);
static aErr sRemoveFromList(aMemPoolObject** ppList, 
			    aMemPoolObject* pObj);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sAddBlock
 */

aErr sAddBlock(aMemPool* pMP)
{  
  aMemPoolBlock*  pBlock;
  unsigned int i;
  aMemPoolObject* pObj;

  aAssert(pMP);

  /* allocate the new block of object storage */
  pBlock = (aMemPoolBlock*)aMemAlloc((aMemSize)pMP->nBlockSize);
  if (pBlock == NULL)
    return(aErrMemory);

#ifdef aDBG_INIT
  /* for debug slam the storage with a known value */
  aaMemSet(pBlock, FREE_MEM_VAL, pMP->nBlockSize);
#endif /* aDBG_INIT */
 
  /* link the block into the head of the memory pool's 
   * block chain */
  pBlock->pNextBlock = pMP->pStorage;
  pMP->pStorage = pBlock;

  /* put all the new block's objects on the free list */
  pObj = &pBlock->firstObject;
  for (i = 0; i < pMP->nBlockObjects; i++) {
    sAddToList(&pMP->pFreeList, pObj);
    pObj = (aMemPoolObject*)(((char*) pObj) + pMP->nMemObjectSize);
  }
  
  return(aErrNone);

} /* end of sAddBlock static local routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sAddToList
 */

aErr sAddToList(aMemPoolObject** ppList, 
		  aMemPoolObject* pObj)
{
  aAssert(ppList);
  aAssert(pObj);

#ifdef aLEAKCHECK
  if (*ppList) {
    /* be sure the list is valid */
    aAssert((*ppList)->links.pPrevObject == NULL);

    /* previous link */
    (*ppList)->links.pPrevObject = pObj;
  }
  pObj->links.pPrevObject = (aMemPoolObject*)NULL;
#endif /* aLEAKCHECK */
 
  /* forward link */
  pObj->links.pNextObject = *ppList;

#ifdef aLEAKCHECK
  if (*ppList)
    (*ppList)->links.pPrevObject = pObj;
#endif /* aLEAKCHECK */

  /* insert */
  *ppList = pObj;

  return(aErrNone);

} /* end of sAddToList static local routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRemoveFromList
 */

aErr sRemoveFromList(aMemPoolObject** ppList, 
		       aMemPoolObject* pObj)
{
  aAssert(ppList);
  aAssert(pObj);

#ifdef aLEAKCHECK
  if (pObj->links.pPrevObject == NULL) {
    /* removing head of list */
    aAssert(pObj == *ppList);
    *ppList = (*ppList)->links.pNextObject;
    if (*ppList)
      (*ppList)->links.pPrevObject = (aMemPoolObject*)NULL;
  } else {
    aMemPoolObject* pPrev = pObj->links.pPrevObject;
    aMemPoolObject* pNext = pObj->links.pNextObject;
    aAssert(pPrev);
    pPrev->links.pNextObject = pNext;
    if (pNext)
      pNext->links.pPrevObject = pPrev;
  }
#else /* aLEAKCHECK */
  aAssert(pObj == *ppList);
  *ppList = (*ppList)->links.pNextObject;
#endif /* aLEAKCHECK */

  return(aErrNone);

} /* end of sRemoveFromList static local routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemPoolInternal_Create
 *
 *
 * This routine creates a MemPool object.  This allows 
 * reuse of fixed sized memory objects.  The objects are 
 * allocated in blocks.  The number of allocated blocks 
 * never decreases.  Using this for transient, like sized, 
 * objects helps avoid fragmentation
 */

aErr aMemPoolInternal_Create(aMemSize objectSize,
		     	     aMemSize blockSize,
		     	     aMemPool** ppMP)
{
  aMemPool* pMP;
  
  if (ppMP == NULL)
    return aErrParam;

  /* sanity check */
  aAssert(objectSize >= 1);
  aAssert(blockSize >= 1);
  aAssert(ppMP);

  /* build the mem pool structure */
  pMP = (aMemPool*)aMemAlloc(sizeof(aMemPool));
  if (pMP == NULL)
    return aErrMemory;

  /* setup the MemPool structure */
  pMP->nMemObjectSize = (aMemSize)(objectSize + sizeof(aMemPoolLinks));
  pMP->nBlockObjects = blockSize;
  pMP->nBlockSize = (aMemSize)(pMP->nMemObjectSize 
  			       * pMP->nBlockObjects 
  			       + sizeof(aMemPoolBlock*));
  pMP->pStorage = (aMemPoolBlock*)NULL;
  pMP->pFreeList = (aMemPoolObject*)NULL;
#ifdef aLEAKCHECK
  pMP->pAllocatedList = (aMemPoolObject*)NULL;
#endif /* aLEAKCHECK */

  *ppMP = pMP;

  return aErrNone;

} /* MemPool_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemPoolInternal_Alloc
 */

aErr aMemPoolInternal_Alloc(aMemPool* pMP,
		    	    void** ppObj)
{
  aErr error;
  aMemPoolObject* pObj;

  aAssert(pMP);
  aAssert(ppObj);

  /* ensure we have a free block to work with */
  if (!pMP->pFreeList) {
    error = sAddBlock(pMP);
    if (error != aErrNone)
      return error;
  }
  
  pObj = pMP->pFreeList;

  /* remove the object at the head of the free list 
   * which is now in use
   */
  error = sRemoveFromList(&pMP->pFreeList, pObj);

#ifdef aLEAKCHECK
  /* add the object to the allocated list */
  sAddToList(&pMP->pAllocatedList, pObj);
#endif /* aLEAKCHECK */
  
  /* set the "allocated" memory pointer */
  *ppObj = &pObj->data;
  
  return error;
  
} /* aMemPoolInternal_Alloc */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemPoolInternal_Free
 */

aErr aMemPoolInternal_Free(aMemPool* pMP,
		   	   void* pObj)
{
  aMemPoolObject* pMemObj = NULL;

  aAssert(pMP);
  aAssert(pObj);

  pMemObj = (aMemPoolObject*)(((char*)pObj) - 
			       sizeof(aMemPoolLinks));

#ifdef aLEAKCHECK
  /* remove the element from the list of allocated objects */
  sRemoveFromList(&pMP->pAllocatedList, pMemObj);
#endif /* aLEAKCHECK */

#ifdef aDBG_INIT
  /* slam the memory to a known value for debuging */
  aMemSet(pMemObj, FREE_MEM_VAL, pMP->nMemObjectSize);
#endif /* aDBG_INIT */

  return(sAddToList(&pMP->pFreeList, pMemObj));

} /* aMemPoolInternal_Free */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemPoolInternal_Destroy
 */

aErr aMemPoolInternal_Destroy(aMemPool* pMP)
{
  aMemPoolBlock* pBlock;

  aAssert(pMP);

#ifdef aLEAKCHECK
  if (pMP->pAllocatedList) {
    aMemPoolObject* pTemp = pMP->pAllocatedList;
    while (pTemp) {
      char msg[1000];
      char num[20];
      aStringCopySafe(msg, 1000, "Memory Leak! Pool: ");
      aStringFromInt(num, pMP);
      aStringCatSafe(msg, 1000, num);
      aStringFromInt(num, pTemp);
      aStringCatSafe(msg, 1000, num);
      aDebugAlert(msg);
      pTemp = pTemp->links.pNextObject;
    }
  }
  aAssert(pMP->pAllocatedList == NULL);
#endif /* aLEAKCHECK */
  
  /* clean up the storage blocks */
  while(pMP->pStorage) {
    pBlock = pMP->pStorage;
    pMP->pStorage = pBlock->pNextBlock;
    aMemFree((aMemPtr)pBlock);
  }

  aMemFree((aMemPtr)pMP);

  return(aErrNone);

} /* aMemPoolInternal_Destroy */
