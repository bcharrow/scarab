/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aMemPoolInternal.h					   */
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

#ifndef _aMemPoolInternal_H_
#define _aMemPoolInternal_H_

#include "aIO.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

typedef struct aMemPoolLinks {
  struct aMemPoolObject*	pNextObject;
#ifdef aLEAKCHECK
  struct aMemPoolObject*	pPrevObject;
#endif /* aLEAKCHECK */
} aMemPoolLinks;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

typedef struct aMemPoolObject {
  aMemPoolLinks			links;
  char				data;
} aMemPoolObject;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

typedef struct aMemPoolBlock {
  struct aMemPoolBlock*		pNextBlock;
  aMemPoolObject		firstObject;
} aMemPoolBlock;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

typedef struct aMemPool {
  aMemSize			nMemObjectSize;
  aMemSize			nBlockObjects;
  aMemSize			nBlockSize;
  aMemPoolBlock*		pStorage;
  aMemPoolObject*		pFreeList;
#ifdef aLEAKCHECK
  aMemPoolObject*		pAllocatedList;
#endif /* aLEAKCHECK */
} aMemPool;

aErr aMemPoolInternal_Create(aMemSize objectSize,
		     	     aMemSize blockSize,
			     aMemPool** ppMP);
aErr aMemPoolInternal_Alloc(aMemPool* pMP,
		    void** ppObj);
aErr aMemPoolInternal_Free(aMemPool* pMP,
		   void* pObj);
aErr aMemPoolInternal_Destroy(aMemPool* pMP);

#define aAssert_aMemPoolSize(p, s)				   \
  aAssert(p->nMemObjectSize - sizeof(aMemPoolLinks) == s);

#endif /* _aMemPoolInternal_H_ */
