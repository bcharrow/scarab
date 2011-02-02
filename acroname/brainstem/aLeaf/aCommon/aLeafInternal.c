/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aLeafInternal.h					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent TEA compiler  */
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

#include "aVersion.h"
#include "aLeafInternal.h"
#include "aLeaf.h"
#include "aLeafText.h"
#include "aTEA.h"
#include "aParseErr.h"
#include "aLAST.h"
#include "aCCMap.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sLeafInternal_Error(tkError error,
				const unsigned int nLine,
				const unsigned int nColumn,
				const unsigned int nData,
				const char* data[],
				void* errProcRef);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLeafInternal_Error
 */

aErr sLeafInternal_Error(tkError error,
			 const unsigned int nLine,
			 const unsigned int nColumn,
			 const unsigned int nData,
			 const char* data[],
			 void* errProcRef)
{
  aErr leafErr = aErrNone;
  aLeaf* pLeaf = (aLeaf*)errProcRef;
  char msg[100];

  aStream_WriteLine(aStreamLibRef(pLeaf->errStream),
  		    pLeaf->errStream,
  		    "leaf error:",
  		    &leafErr);

  /* show where the error occured */
  if (leafErr == aErrNone) {
    char num[10];
    aStringCopy(msg, " ");
    /* add the filename if present */
    if (nData > 0) {
      aStringCat(msg, data[0]);
      aStringCat(msg, ", ");
    }
    aStringCat(msg, "line ");
    aStringFromInt(num, nLine);
    aStringCat(msg, num);
    aStringCat(msg, ", column ");
    aStringFromInt(num, nColumn);
    aStringCat(msg, num);    
    aStream_WriteLine(aStreamLibRef(pLeaf->errStream),
  		      pLeaf->errStream,
  		      msg,
  		      &leafErr);
  }

  /* show the actual error */  
  if (leafErr == aErrNone) {
    aStringCopy(msg, " ");

    switch(error) {

    case tkErrUntermCmnt:
      aStringCat(msg, aTE_UNTERMINATED_COMMENT);
      break;

    case tkErrIncludeNotFnd:
      aStringCat(msg, aTE_INCLUDE_NOT_FOUND);
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;

    case tkErrDuplicateDefine:
      aStringCat(msg, aTE_SYMBOL_DEFINED);
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;

    case tkErrBadArgumentList:
      aStringCat(msg, aTE_BAD_ARG_LIST);
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;

    case tkErrInvalidChar:
      aStringCat(msg, aTE_INVALID_CHAR);
      if (nData > 1) {
        char num[4];
        const char* pData = data[1];
        unsigned char c = (unsigned char)*pData;
        aStringFromInt(num, c);
        aStringCat(msg, num);
      }
      break;

    default:
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;

    } /* switch */

    aStream_WriteLine(aStreamLibRef(pLeaf->errStream),
  		      pLeaf->errStream,
  		      msg,
  		      &leafErr);
  }

  /* increment the error count */
  if (pLeaf->vpLAST)
    ((aLAST*)pLeaf->vpLAST)->nErrors++;

  return leafErr;

} /* sLeafInternal_Error */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLeafInternal_Create
 */

aErr aLeafInternal_Create(aLeaf** ppLeaf)
{
  aErr leafErr = aErrNone;
  aLeaf* pLeaf = NULL;

  if (ppLeaf == NULL)
    leafErr = aErrParam;

  if (leafErr == aErrNone) {
    pLeaf = (aLeaf*)aMemAlloc(sizeof(aLeaf));
    if (pLeaf == NULL)
      leafErr = aErrParam;
    else {
      aBZero(pLeaf, sizeof(aLeaf));
      pLeaf->check = aLEAFCHECK;
      if (leafErr != aErrNone)
        aLeafInternal_Destroy(pLeaf);
    }
  }

  if (leafErr == aErrNone) {
    *ppLeaf = pLeaf;
  }

  return leafErr;

} /* aLeafInternal_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLeafInternal_Compile
 */

aErr aLeafInternal_Compile(aLeaf* pLeaf,
			   aStreamRef source,
			   const char* sourceName,
			   int flags,
			   aStreamRef result,
			   aStreamRef out,
			   aStreamRef err)
{
  aErr leafErr = aErrNone;
  aTokenizerRef tokenizerRef = NULL;
  aLAST* pLAST;

  /* set up the error stream for reporting errors */
  pLeaf->errStream = err;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * display the welcome message 
   */
  if (out != NULL) {
    aStream_WriteLine(aStreamLibRef(out), out, "", NULL);
    aStream_WriteLine(aStreamLibRef(out), out, 
    		      aLEAF_COMPILE_WELCOME1, NULL);
    aStream_WriteLine(aStreamLibRef(out), out, 
    		      aLEAF_COMPILE_WELCOME2, NULL);
    aStream_WriteLine(aStreamLibRef(out), out, 
    		      aLEAF_VERSION_LINE, NULL);
    aStream_WriteLine(aStreamLibRef(out), out, 
    		      aCOPYRIGHT_LINE, NULL);
    aStream_WriteLine(aStreamLibRef(out), out, "", NULL);
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * build a tokenizer if we are generating code or preprocessing
   */
  if (flags & (fLeafGenerateCode 
  	       | fLeafPreprocess
  	       | fLeafAST)) {
    if (leafErr == aErrNone)
      aTokenizer_Create(aStreamLibRef(source), 
      			source, 
      			sourceName, 
      			aFileAreaUser, 
      			sLeafInternal_Error, 
      			pLeaf, 
      			&tokenizerRef, 
      			&leafErr);
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * just preprocess
   */
  if (flags & fLeafPreprocess) {
    aErr ppErr = aErrNone;
    aToken* pToken;
    
    aAssert(tokenizerRef);
    while ((ppErr == aErrNone) && 
    	   !aTokenizer_Next(aStreamLibRef(source), 
    	   		    tokenizerRef, 
    	   		    &pToken, 
    	   		    &ppErr)) {
      ppErr = aToken_Output(pToken, "", result);
      if (ppErr == aErrNone)
        aTokenizer_Dispose(aStreamLibRef(source), 
        		   tokenizerRef, 
      			   pToken, &ppErr);
    }

    if (ppErr != aErrEOF)
      aStream_WriteLine(aStreamLibRef(out), out, 
      			aLEAF_PP_FAILURE, NULL);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * run the compiler
   */
  } else {
    if (leafErr == aErrNone)
      leafErr = aLAST_Create(aStreamLibRef(source), 
      			     &pLAST, 
      			     sLeafInternal_Error,
      			     pLeaf);
 
    /* parse into a syntax tree */
    if (leafErr == aErrNone) {
      pLeaf->vpLAST = pLAST;
      leafErr = aLAST_Parse(pLAST, tokenizerRef, err);
    }

    /* fold constants */
    if (leafErr == aErrNone)
      leafErr = aLAST_Optimize(pLAST, err);

#ifdef aASTOUTPUT
    if ((leafErr == aErrNone) &&
        (flags & fLeafAST))
      leafErr = aAST_Output((aAST*)pLAST, out, err, result);
#endif /* aASTOUTPUT */

    if ((leafErr == aErrNone) &&
        (flags & fLeafGenerateCode))
      leafErr = aLAST_Generate(pLAST, result);

    /* reset the error if it was just a parser error */
    if (leafErr == aErrParse)
      leafErr = aErrNone;

    if (leafErr == aErrNone)
      leafErr = aLAST_Destroy(pLAST);

    pLeaf->vpLAST = NULL;
  }

  /* clean up */
  if (tokenizerRef != NULL)
    aTokenizer_Destroy(aStreamLibRef(source), tokenizerRef, NULL);

  return leafErr;

} /* aLeafInternal_Compile */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLeafInternal_Destroy
 */

aErr aLeafInternal_Destroy(aLeaf* pLeaf)
{
  aErr leafErr = aErrNone;

  aVALIDLEAF(pLeaf);

  if (leafErr == aErrNone) {
    pLeaf->check = 0;
    aMemFree((aMemPtr)pLeaf);
  }
  
  return leafErr;

} /* aLeafInternal_Destroy */

