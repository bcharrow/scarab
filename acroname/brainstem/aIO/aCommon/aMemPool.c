/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aMemPool.c						   */
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


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemPool_Create
 *
 *
 * This routine creates a MemPool object.  This allows 
 * reuse of fixed sized memory objects.  The objects are 
 * allocated in blocks.  The number of allocated blocks 
 * never decreases.  Using this for transient, like sized, 
 * objects helps avoid fragmentation
 */

aLIBRETURN aMemPool_Create(aIOLib ioRef,
			   aMemSize objectSize,
		     	   aMemSize blockSize,
		     	   aMemPoolRef* pPoolRef,
		     	   aErr* pErr)
{
  aErr poolErr = aMemPoolInternal_Create(objectSize, blockSize,
  					 (aMemPool**)pPoolRef);
  if (pErr != NULL)
    *pErr = poolErr;

  return (aLIBRETURN)(poolErr != aErrNone);

} /* MemPool_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemPool_Alloc
 */

aLIBRETURN aMemPool_Alloc(aIOLib ioRef,
			  aMemPoolRef poolRef,
		    	  void** ppObj,
		     	  aErr* pErr)
{
  aErr poolErr = aMemPoolInternal_Alloc((aMemPool*)poolRef, ppObj);

  if (pErr != NULL)
    *pErr = poolErr;

  return (aLIBRETURN)(poolErr != aErrNone);
 
} /* aMemPool_Alloc */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemPool_Free
 */

aLIBRETURN aMemPool_Free(aIOLib ioRef,
			 aMemPoolRef poolRef,
		   	 void* pObj,
		     	 aErr* pErr)
{
  aErr poolErr = aMemPoolInternal_Free((aMemPool*)poolRef, pObj);

  if (pErr != NULL)
    *pErr = poolErr;

  return (aLIBRETURN)(poolErr != aErrNone);

} /* aMemPool_Free */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemPool_Destroy
 */

aLIBRETURN aMemPool_Destroy(aIOLib ioRef,
			    aMemPoolRef poolRef,
		     	    aErr* pErr)
{
  aErr poolErr = aMemPoolInternal_Destroy((aMemPool*)poolRef);

  if (pErr != NULL)
    *pErr = poolErr;

  return (aLIBRETURN)(poolErr != aErrNone);

} /* aMemPool_Destroy */

