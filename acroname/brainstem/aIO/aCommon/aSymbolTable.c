/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSymbolTable.c						   */
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
#include "aSymbolTableInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSymbolTable_Create
 */

aLIBRETURN aSymbolTable_Create(aIOLib ioRef,
			       aSymbolTableRef* pSymRef,
		     	   aErr* pErr)
{
  aErr symErr = aSymbolTableInternal_Create((aSymbolTable**)pSymRef);
  if (pErr != NULL)
    *pErr = symErr;

  return (aLIBRETURN)(symErr != aErrNone);

} /* aSymbolTable_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSymbolTable_Insert
 */

aLIBRETURN aSymbolTable_Insert(aIOLib ioRef,
			       aSymbolTableRef symbolTableRef,
			       const char* identifier,
			       void* pData,
  			       symDataDeleteProc deleteProc,
  			       void* deleteRef,
		     	       aErr* pErr)
{
  aErr symErr = aSymbolTableInternal_Insert((aSymbolTable*)symbolTableRef, 
  					    identifier,
  					    pData,
  					    deleteProc,
  					    deleteRef);
  if (pErr != NULL)
    *pErr = symErr;

  return (aLIBRETURN)(symErr != aErrNone);
 
} /* aSymbolTable_Insert */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSymbolTable_Find
 */

aLIBRETURN aSymbolTable_Find(aIOLib ioRef,
			     aSymbolTableRef symbolTableRef,
		       	     const char* identifier,
		       	     void** ppFoundData,
		     	     aErr* pErr)
{
  aErr symErr = aSymbolTableInternal_Find((aSymbolTable*)symbolTableRef,
  					  identifier,
  					  ppFoundData);

  if (pErr != NULL)
    *pErr = symErr;

  return (aLIBRETURN)(symErr != aErrNone);

} /* aSymbolTable_Find */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSymbolTable_Destroy
 */

aLIBRETURN aSymbolTable_Destroy(aIOLib ioRef,
			    aSymbolTableRef poolRef,
		     	    aErr* pErr)
{
  aErr symErr = aSymbolTableInternal_Destroy((aSymbolTable*)poolRef);

  if (pErr != NULL)
    *pErr = symErr;

  return (aLIBRETURN)(symErr != aErrNone);

} /* aSymbolTable_Destroy */

