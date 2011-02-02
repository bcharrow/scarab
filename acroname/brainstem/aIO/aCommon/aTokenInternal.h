/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTokenInternal.h					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: tokenizer with preprocessor.			   */
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

#ifndef _aToken_H_
#define _aToken_H_

#include "aIO.h"
#include "aParseErr.h"
#include "aCCMap.h"
#include "aParseFrame.h"
#include "aMemPoolInternal.h"
#include "aSymbolTableInternal.h"

#define aTOKENSCANBLOCKSIZE		40

#define tkMacroMask	(tkType)0x80
#define tkReference	(tkType)10

/* warning, the structure below must be an exact super set of 
 * the aToken structure found in aIO.h */
#define aINPLACESTRINGMAX	(aMAXIDENTIFIERLEN                  \
                                 - sizeof(char*)                    \
                                 - sizeof(aStreamRef))
typedef struct aPrivateString {
  char*			string;
  aStreamRef		stringBuffer;
  char			storage[aINPLACESTRINGMAX];
} aPrivateString;

typedef struct aPrivateToken {
  tkType		eType;
  union {
    int			integer;
    float		floatVal;
    char		identifier[aMAXIDENTIFIERLEN];
    aPrivateString	s;
    char		special;
  } v;
} aPrivateToken;

typedef struct aTokenInternal {
  aPrivateToken			t;
  unsigned int			line;
  unsigned int			column;
  aParseFrame*			pParseFrame;
  struct aTokenInternal*	pNext;
} aTokenInternal;

typedef struct aTokenList {
  aTokenInternal*		pHead;
  aTokenInternal*		pTail;
  aMemPool*			pTokenPool;
  struct aTokenList*		pNext;
  void*				pRef; /* MRW */
} aTokenList;

typedef struct aPPStack {
  char				identifier[aMAXIDENTIFIERLEN];
  aBool				bHides;
  struct aPPStack* 		pNext;
} aPPStack;

typedef struct aTokenizer {
  aIOLib			ioRef;
  ccType*			ccMap;
  aParseFrame*			pInputFrame;
  aParseFrame*			pParseFrameRoot;
  aMemPool*			pTokenPool;
  aTokenList*			pTokenList;
  aSymbolTable*			pSymbolTable;
  aFileArea			eIncludeArea;
  aMemPool*			pPPPool;
  int				nPPHide;
  aPPStack*			pPPStack;
  parseErr			parseError;
  aErr				err;

  char*				pScanBuf;
  unsigned long			nScanCur;
  unsigned long			nScanLen;
  aBool				bScanDone;

  aTokenErrProc			errProc;
  void*				errProcRef;

} aTokenizer;



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * token list routines
 */

aErr aTokenList_Create(aMemPool* pTokenPool,
		       aTokenList** ppList);

aErr aTokenList_Add(aTokenList* pList,
		    aTokenInternal* pTokenInternal);

aErr aTokenList_AddFirst(aTokenList* pList,
		         aTokenInternal* pTokenInternal);

aErr aTokenList_AddCopy(aTokenList* pList,
		        const aTokenInternal* pTokenInternal,
		        const unsigned int nLine,
		        const unsigned int nColumn);

aErr aTokenList_AddListCopy(aTokenList* pList,
		            aTokenList* pAddList,
		            unsigned int line,
		            unsigned int column);

aErr aTokenList_GetFirst(aTokenList* pList,
		         aTokenInternal** ppToken);

aErr aTokenList_Empty(aTokenList* pList);

aErr aTokenList_Destroy(aTokenList* pList);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * token routines
 */

aErr aTokenInternal_GetInfo(const aToken* pToken,
		            aTokenInfo* pTokenInfo);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * tokenizer routines
 */

aErr aTokenizerInternal_Create(aStreamRef tokenStream,
			       const char* streamName,
		       	       aFileArea eIncludeArea,
			       ccType* ccmap,
			       aTokenErrProc errProc,
			       void* errProcRef,
		       	       aTokenizer** ppTokenizer);

void aTokenizerInternal_OutputError(aTokenizer* pTokenizer,
			    char* description,
			    aStreamRef errorStream);

aErr aTokenizerInternal_Next(aTokenizer* pTokenizer,
		             aToken** ppToken);

aErr aTokenizerInternal_PushBack(aTokenizer* pTokenizer,
		      	  aToken* pToken);

aErr aTokenizerInternal_Dispose(aTokenizer* pTokenizer,
		        aToken* pToken);

aErr aTokenizerInternal_Destroy(aTokenizer* pTokenizer);

#endif /* _aToken_H_ */

