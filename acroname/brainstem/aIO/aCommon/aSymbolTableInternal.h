/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSymbolTableInternal.h					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: definitions of symbol table object.                */
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

#ifndef _aSymbolTableInternal_H_
#define _aSymbolTableInternal_H_

#include "aMemPoolInternal.h"


typedef struct aSym {
  char			identifier[aMAXIDENTIFIERLEN];
  struct aSym*		pLeft;
  struct aSym*		pRight;
  void*			pData;
  symDataDeleteProc	deleteProc;
  void*			deleteRef;
} aSym;

typedef struct aSymbolTable {
  aSym*			pRootNode;
  aMemPool*		pMemPool;
} aSymbolTable;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * routines
 */

aErr aSymbolTableInternal_Create(aSymbolTable** ppTable);
aErr aSymbolTableInternal_Insert(aSymbolTable* pTable,
			         const char* identifier,
			         void* pData,
  			         symDataDeleteProc deleteProc,
  			         void* deleteRef);
aErr aSymbolTableInternal_Find(aSymbolTable* pTable,
		               const char* identifier,
		               void** ppFoundData);
aErr aSymbolTableInternal_Destroy(aSymbolTable* pTable);

#endif /* _aSymbolTableInternal_H_ */
