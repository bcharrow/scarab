/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aMemLeakDebug.c					   */
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

/* fools compilers that don't want an empty file */
int aMemLeakDebugFile;

#ifdef aLEAKCHECK

#define aMAXLEAKCHECKFILENAME	128

#include "aOSDefs.h"
#include "aMemLeakDebug.h"

#ifdef aMAC
#define aLeakAlloc(s)		NewPtr((long)(s))
#define aLeakFree(s)		DisposePtr(s)
#endif /* aMAC */

#ifdef aPALM
#define aLeakAlloc(s)		MemPtrNew(s)
#define aLeakFree(s)		MemPtrFree(s)
#endif /* aPALM */

#ifdef aWIN
#define aLeakAlloc(s)		malloc(s)
#define aLeakFree(s)		free(s)
#endif /* aWIN */

#ifdef aWINCE
#include <string.h>
#define aLeakAlloc(s)		malloc(s)
#define aLeakFree(s)		free(s)
#endif /* aWINCE */

#ifdef aUNIX
#define aLeakAlloc(s)		malloc(s)
#define aLeakFree(s)		free(s)
#endif /* aUNIX */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * block for tracking memory allocations
 */

typedef struct aLeakCheckBlock {
  struct aLeakCheckBlock*	pNext;
  char				filename[aMAXLEAKCHECKFILENAME];
  int				linenum;
  int				allocnum;
  aMemSize			size;
} aLeakCheckBlock;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * globals to maintain allocated and freed memory blocks
 */
 
aLeakCheckBlock* g_pLeakCheckAlloc = (aLeakCheckBlock*)NULL;
aLeakCheckBlock* g_pLeakCheckFree = (aLeakCheckBlock*)NULL;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLeakCheckAlloc
 */

aMemPtr aLeakCheckAlloc(
  aMemSize size,
  const char* filename,
  int linenum
)
{
  aLeakCheckBlock* pNew;
  char* p;
  static int s_nLeakCheckAlloc = 0;

  pNew = (aLeakCheckBlock*)aLeakAlloc(sizeof(aLeakCheckBlock) + size);
  if (pNew == NULL)
    return NULL;
  
  aStringCopySafe(pNew->filename, aMAXLEAKCHECKFILENAME, filename);
  pNew->linenum = linenum;
  pNew->size = size;
  pNew->allocnum = ++s_nLeakCheckAlloc;

#if 0
  /* debug checking to force an error on a particular allocation */
  if (pNew->allocnum == 7754) {
    char* pooh = NULL;
    *pooh = 0; /* should force an error */
  }
#endif

  /* link it into the chain of allocated blocks */
  pNew->pNext = g_pLeakCheckAlloc;
  g_pLeakCheckAlloc = pNew;

  /* compute the offset to the block's data area */
  p = (char*)pNew;
  p += sizeof(aLeakCheckBlock);
  
#ifdef aDEBUG
  /* slam the memory with a set value to make sure we don't
   * rely on zeros in the memory. */
  {
    aMemSize i;
    char *t = p;
    for (i = 0; i < size; i++) {
      *t++ = 0x1E;
    }
  }
#endif /* aDEBUG */

  return(p);
  
} /* end of aLeakCheckAlloc */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLeakCheckFree
 */

void aLeakCheckFree(
  aMemPtr p,
  const char* filename,
  int linenum
)
{
  char* pMem;
  aLeakCheckBlock* pRec;
  aLeakCheckBlock* pTemp;
  aLeakCheckBlock* pPrev;

  /* compute the actual block header address */
  pMem = (char*)p;
  pMem -= sizeof(aLeakCheckBlock);  
  pRec = (aLeakCheckBlock*)pMem;

  /* find the block in the allocated list */
  pPrev = (aLeakCheckBlock*)NULL;
  pTemp = g_pLeakCheckAlloc;
  
  do {

    /* found it */
    if (pTemp == pRec) {
    
      /* pull it out of the list */
      if (pPrev == NULL) {
        /* special case at head of list */
        g_pLeakCheckAlloc = pRec->pNext;
      } else {
        pPrev->pNext = pTemp->pNext;
      }
      break;
    }
    /* advance to the next in the list */
    if (pTemp) {
      pPrev = pTemp;
      pTemp = pTemp->pNext;
    }
  } while (pTemp != NULL);

  if (pTemp == NULL) {
    /* error, freeing memory not allocated */
    char msg[200];
    char num[10];
    aStringCopySafe(msg, sizeof(msg), "Freeing Unallocated Memory: ");
    aStringCatSafe(msg, sizeof(msg), filename);
    aStringCatSafe(msg, sizeof(msg), ", line ");
    aStringFromInt(num, linenum);
    aStringCatSafe(msg, sizeof(msg), num);
    aDebugAlert(msg);
  } else {
    /* just toss the freed memory record */
    aLeakFree((aMemPtr)pTemp);
  }

} /* end of aLeakCheckFree */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLeakCheckCleanup
 */

void aLeakCheckCleanup(void)
{
  aLeakCheckBlock* pTemp = g_pLeakCheckAlloc;
  char msg[200];

  int i = 0;
  while(pTemp != NULL) {
    /* error, freeing memory not freed */
    char num[10];
    aStringCopySafe(msg, sizeof(msg), "Memory Leak: ");
    aStringCatSafe(msg, sizeof(msg), pTemp->filename);
    aStringCatSafe(msg, sizeof(msg), ", line ");
    aStringFromInt(num, pTemp->linenum);
    aStringCatSafe(msg, sizeof(msg), num);
    aStringCatSafe(msg, sizeof(msg), ", ");
    aStringFromInt(num, pTemp->size);
    aStringCatSafe(msg, sizeof(msg), num);
    aStringCatSafe(msg, sizeof(msg), " bytes, allocation ");
    aStringFromInt(num, pTemp->allocnum);
    aStringCatSafe(msg, sizeof(msg), num);
    aDebugAlert(msg);
    pTemp = pTemp->pNext;
    i++;
  }

  if (i) {  
    aSNPRINTF(msg, 200, "There were %d total memory leaks", i);
    aDebugAlert(msg);
  }

} /* aLeakCheckCleanup */

#endif /* aLEAKCHECK */

