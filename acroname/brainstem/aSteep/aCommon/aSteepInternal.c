/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSteepInternal.h					   */
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

#include "aUtil.h"
#include "aVersion.h"
#include "aSteepInternal.h"
#include "aSteep.h"
#include "aSteepGen.h"
#include "aSteepText.h"
#include "aTEA.h"
#include "aParseErr.h"
#include "aAST.h"
#include "aSAST.h"
#include "aCCMap.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sSteepInternal_Error(tkError error,
				 const unsigned int nLine,
				 const unsigned int nColumn,
				 const unsigned int nData,
				 const char* data[],
				 void* errProcRef);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSteepInternal_Error
 */

aErr sSteepInternal_Error(tkError error,
			  const unsigned int nLine,
			  const unsigned int nColumn,
			  const unsigned int nData,
			  const char* data[],
			  void* errProcRef)
{
  aErr steepErr = aErrNone;
  aSteep* pSteep = (aSteep*)errProcRef;
  char msg[100];
  char num[10];

  aStream_WriteLine(aStreamLibRef(pSteep->errStream),
  		    pSteep->errStream,
  		    "steep error:",
  		    &steepErr);

  /* show the error location */
  
  /* the filename is always the first data parameter */
  if ((steepErr == aErrNone) && (nData > 0)) {
    aStringCopy(msg, " file: ");
    aStringCat(msg, data[0]);
    aStream_WriteLine(aStreamLibRef(pSteep->errStream),
  		      pSteep->errStream, msg, &steepErr);
  }

  if (steepErr == aErrNone) {        
    aStringCopy(msg, " line: ");
    aStringFromInt(num, nLine);
    aStringCat(msg, num);
    aStream_WriteLine(aStreamLibRef(pSteep->errStream), 
        	      pSteep->errStream, msg, &steepErr);
  }
    
  if (steepErr == aErrNone) {
    aStringCopy(msg, " character: ");
    aStringFromInt(num, nColumn);
    aStringCat(msg, num);
    aStream_WriteLine(aStreamLibRef(pSteep->errStream), 
        	      pSteep->errStream, msg, &steepErr);
  }

  /* show the actual error */  
  if (steepErr == aErrNone) {
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
      aStringCat(msg, aSE_SYMBOL_DEFINED);
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;

    case tkErrBadArgumentList:
      aStringCat(msg, aSE_BAD_ARG_LIST);
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;

    case tkErrMissingParen:
      aStringCat(msg, aSE_MISSING_PAREN);
      break;

    case tkErrInvalidChar:
      aStringCat(msg, aSE_INVALID_CHAR);
      if (nData > 1) {
        char n[4];
        aStringFromInt(n, (unsigned char)(*(data[1])));
        aStringCat(msg, n);
      }
      break;

    default:
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;

    } /* switch */    

    aStream_WriteLine(aStreamLibRef(pSteep->errStream),
  		      pSteep->errStream,
  		      msg,
  		      &steepErr);
  }

  /* increment error count */
  if (pSteep->vpSAST)
    ((aSAST*)pSteep->vpSAST)->nErrors++;

  return steepErr;

} /* sSteepInternal_Error */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepInternal_Create
 */

aErr aSteepInternal_Create(aSteep** ppSteep)
{
  aErr steepErr = aErrNone;
  aSteep* pSteep = NULL;

  if (ppSteep == NULL)
    steepErr = aErrParam;

  if (steepErr == aErrNone) {
    pSteep = (aSteep*)aMemAlloc(sizeof(aSteep));
    if (pSteep == NULL)
      steepErr = aErrParam;
    else {
      aBZero(pSteep, sizeof(aSteep));
      pSteep->check = aSTEEPCHECK;
      if (steepErr != aErrNone)
        aSteepInternal_Destroy(pSteep);
    }
  }

  if (steepErr == aErrNone)
    aSteepOpt_GetLibRef(&pSteep->optLib, &steepErr);

  if (steepErr == aErrNone)
    aSteepGen_GetLibRef(&pSteep->genLib, &steepErr);

  if (steepErr == aErrNone) {
    aTEA_SetOpCodeLengths(pSteep->opLengths);
    *ppSteep = pSteep;
  }

  return steepErr;

} /* aSteepInternal_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepInternal_Compile
 */

aErr aSteepInternal_Compile(aSteep* pSteep,
			    aStreamRef source,
			    const char* sourceName,
			    int flags,
			    aStreamRef result,
			    aStreamRef out,
			    aStreamRef err)
{
  aErr steepErr = aErrNone;
  aTokenizerRef tokenizerRef = NULL;
  aSAST* pSAST;

  /* set up the error stream for reporting errors */
  pSteep->errStream = err;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * display the welcome message 
   */
  if (out != NULL) {
    aStream_WriteLine(aStreamLibRef(out), out, 
    		      "", NULL);
    aStream_WriteLine(aStreamLibRef(out), out, aSTEEP_COMPILE_WELCOME1, NULL);
    aStream_WriteLine(aStreamLibRef(out), out, aSTEEP_COMPILE_WELCOME2, NULL);
    aStream_WriteLine(aStreamLibRef(out), out, aSTEEP_VERSION_LINE, NULL);
    aStream_WriteLine(aStreamLibRef(out), out, aCOPYRIGHT_LINE, NULL);
    aStream_WriteLine(aStreamLibRef(out), out, "", NULL);
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * build a tokenizer if we are generating code or preprocessing
   */
  if (flags & (fSteepGenerateCode 
  	       | fSteepPreprocess
  	       | fSteepSAST)) {
    if (steepErr == aErrNone)
      aTokenizer_Create(aStreamLibRef(source), 
      			source, 
      			sourceName, 
      			aFileAreaUser, 
      			sSteepInternal_Error, 
      			pSteep, 
      			&tokenizerRef, 
      			&steepErr);
  }


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * handle the case of dissembly
   */
  if (flags & fSteepDisassemble) {

    if (steepErr == aErrNone) {
      steepErr = aTEA_Disassemble(source, err, result, 
      				  pSteep->opLengths);
      if (steepErr == aErrNone)
        aStream_Destroy(aStreamLibRef(source), source, &steepErr);
    }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * just preprocess
   */
  } else if (flags & fSteepPreprocess) {
    aErr ppErr = aErrNone;
    aToken* pToken;
    
    aAssert(tokenizerRef);
    while ((ppErr == aErrNone) && 
    	   !aTokenizer_Next(aStreamLibRef(source), tokenizerRef, 
    	   		    &pToken, &ppErr)) {
      ppErr = aToken_Output(pToken, "", result);
      if (ppErr == aErrNone)
        aTokenizer_Dispose(aStreamLibRef(source), tokenizerRef, 
      			   pToken, &ppErr);
    }

    /* MRW 11-28-01 (nice to know if pre-processing fails) */
    if (ppErr != aErrEOF)
      aStream_WriteLine(aStreamLibRef(out), out, aSTEEP_PP_FAILURE, NULL);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * run the compiler
   */
  } else {
    if (steepErr == aErrNone)
      steepErr = aSAST_Create(aStreamLibRef(source), 
      			      &pSAST, 
      			      pSteep->opLengths,
      			      sSteepInternal_Error, 
      			      pSteep);
 
    /* parse into a syntax tree */
    if (steepErr == aErrNone) {
      pSteep->vpSAST = pSAST;
      steepErr = aSAST_Parse(pSAST, tokenizerRef, err);
    }

    /* optimize the tree */
    if ((steepErr == aErrNone)
        && (pSAST->nErrors == 0)
        && (pSAST->pTree))
      aSteepOpt_Optimize(pSteep->optLib, pSAST, &steepErr);

#ifdef aASTOUTPUT
    if ((steepErr == aErrNone) &&
        (flags & fSteepSAST))
      steepErr = aAST_Output((aAST*)pSAST, out, err, result);
#endif /* aASTOUTPUT */

    /* generate code from the SAST */
    if ((steepErr == aErrNone)
        && (flags & fSteepGenerateCode)
        && (pSAST->nErrors == 0)
        && (pSAST->pTree))
      aSteepGen_Generate(pSteep->genLib, pSAST, result, &steepErr);

    /* post a result message */
    if ((steepErr == aErrNone)
        && (out != NULL)) {
      if (pSAST->nErrors == 0) {
        aStream_WriteLine(aStreamLibRef(out), out, 
      			  aSTEEP_SUCCESS, NULL);
      } else {
        char msg[40];
	aSNPRINTF(msg, 40, aSTEEP_FAILED, pSAST->nErrors);
        aStream_WriteLine(aStreamLibRef(out), out, 
      			  "", NULL);
        aStream_WriteLine(aStreamLibRef(out), out, 
      			  msg, NULL);
        
      }
    }

    /* reset the error if it was just a parser error */
    if (steepErr == aErrParse)
      steepErr = aErrNone;

    if (steepErr == aErrNone)
      steepErr = aSAST_Destroy(pSAST);

    /* reset the current SAST (used for error counting) */
    pSteep->vpSAST = NULL;
  }

  /* clean up */
  if (tokenizerRef != NULL)
    aTokenizer_Destroy(aStreamLibRef(source), tokenizerRef, NULL);

  return steepErr;

} /* aSteepInternal_Compile */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepInternal_Destroy
 */

aErr aSteepInternal_Destroy(aSteep* pSteep)
{
  aErr steepErr = aErrNone;

  aVALIDSTEEP(pSteep);

  if (pSteep->genLib)
    aSteepGen_ReleaseLibRef(pSteep->genLib, NULL);

  if (pSteep->optLib)
    aSteepOpt_ReleaseLibRef(pSteep->optLib, NULL);

  if (steepErr == aErrNone) {
    pSteep->check = 0;
    aMemFree((aMemPtr)pSteep);
  }
  
  return steepErr;

} /* aSteepInternal_Destroy */
