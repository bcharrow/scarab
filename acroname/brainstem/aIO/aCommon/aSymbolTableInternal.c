/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSymbolTableInternal.c					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: implementation of symbol table object.             */
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

#include "aSymbolTableInternal.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local routines
 */

static aErr node_destroy(aSym* pNode,
			 aMemPool* pMemPool);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * node_destroy
 */

static aErr node_destroy(aSym* pNode,
			 aMemPool* pMemPool)
{
  aErr nodeErr = aErrNone;

  if ((nodeErr == aErrNone) && (pNode->pLeft != NULL))
    nodeErr = node_destroy(pNode->pLeft, pMemPool);

  if ((nodeErr == aErrNone) && (pNode->pRight != NULL))
    nodeErr = node_destroy(pNode->pRight, pMemPool);

  if ((nodeErr == aErrNone) && (pNode->pData != NULL)) {
    if (pNode->deleteProc != NULL) {
      nodeErr = pNode->deleteProc(pNode->pData, pNode->deleteRef);
    } else {
      aMemFree(pNode->pData);
    }
  }

  if (nodeErr == aErrNone)
    aMemPoolInternal_Free(pMemPool, pNode);

  return(nodeErr);

} /* node_destroy */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSymbolTableInternal_Create
 */

aErr aSymbolTableInternal_Create(aSymbolTable** ppTable)
{
  aErr symErr = aErrNone;
  aSymbolTable* pTable;
  
  aAssert(ppTable);

  pTable = (aSymbolTable*)aMemAlloc(sizeof(aSymbolTable));
  if (pTable == NULL)
    symErr = aErrMemory;

  if (symErr == aErrNone) {
    pTable->pRootNode = NULL;
    symErr = aMemPoolInternal_Create(sizeof(aSym), 16, 
    				     &pTable->pMemPool);
    if (symErr == aErrNone)
      *ppTable = pTable;
  }

  return(symErr);

} /* end of aSymbolTableInternal_Create routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSymbolTableInternal_Insert
 *
 * ppResult is set to the actual node data in the symbol
 * table after inserting
 */

aErr aSymbolTableInternal_Insert(aSymbolTable* pTable,
			 	 const char* identifier,
			 	 void* pData,
			 	 symDataDeleteProc deleteProc,
			 	 void* deleteRef)
{
  aErr symErr;
  aSym* pNewNode;
  aSym* pCur;
  int dir;

  aSym* pTree;

  aAssert(pTable);
  aAssert(identifier);


  symErr = aSymbolTableInternal_Find(pTable, identifier, NULL);
  if (symErr == aErrNotFound) {
    void* p;
    pNewNode = NULL;
    symErr = aMemPoolInternal_Alloc(pTable->pMemPool, &p);
    if (symErr == aErrNone) {
      pNewNode = (aSym*)p;
      aBZero(pNewNode, sizeof(aSym));
      aStringCopySafe(pNewNode->identifier, aMAXIDENTIFIERLEN, identifier);
      pNewNode->pData = pData;
      pNewNode->deleteProc = deleteProc;
      pNewNode->deleteRef = deleteRef;
    }
  }

  if (symErr == aErrNone) {
  
    if (pTable->pRootNode == NULL) {
      pTable->pRootNode = pNewNode;
    } else {
      pCur = pTable->pRootNode;
      do {
        pTree = pCur;
        dir = aStringCompare(identifier, pTree->identifier);
        pCur = (dir < 0) ? pTree->pLeft : pTree->pRight;
      } while (dir && (pCur != NULL));

      /* put the new node into tree */
      if (dir < 0) {
        pTree->pLeft = pNewNode;
      } else if (dir > 0) {
        pTree->pRight = pNewNode;
      } else {
        symErr = aErrDuplicate;
      }
    }
  }

  return(symErr);

} /* end of aSymbolTableInternal_Insert routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSymbolTableInternal_Find
 *
 * Try to find an item in the symbol table.  If the 
 * return pointer is NULL, this routine just reports 
 * whether the symbol was found otherwise, it sets
 * a point to the found symbol.
 */

aErr aSymbolTableInternal_Find(aSymbolTable* pTable,
		               const char* identifier,
		               void** ppFoundData)
{
  aSym* pCur;
  int dir;

  aAssert(pTable);
  aAssert(identifier);

  pCur = pTable->pRootNode;
  
  while (pCur) {
    dir = aStringCompare(identifier, pCur->identifier);
    if (dir < 0)
      pCur = pCur->pLeft;
    else if (dir > 0)
      pCur = pCur->pRight;
    else {
      if (ppFoundData)
	*ppFoundData = pCur->pData;
      return(aErrNone);
    }
  } /* while pCur */
    
  return(aErrNotFound);

} /* end of aSymbolTableInternal_Find routine */
		   


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSymbolTableInternal_Destroy
 */

aErr aSymbolTableInternal_Destroy(aSymbolTable* pTable)
{
  aErr symErr = aErrNone;

  aAssert(pTable);
  aAssert(pTable->pMemPool);
  
  if (pTable->pRootNode != NULL)
    symErr = node_destroy(pTable->pRootNode, pTable->pMemPool);

  if (symErr == aErrNone)
    symErr = aMemPoolInternal_Destroy(pTable->pMemPool);

  if (symErr == aErrNone)
    aMemFree((aMemPtr)pTable);
  
  return(symErr);

} /* end of aSymbolTableInternal_Destroy routine */
