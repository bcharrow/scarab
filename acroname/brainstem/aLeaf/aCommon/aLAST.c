/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aLAST.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of an LAST (annotated syntax tree)  */
/*		for the TEA compiler. 		                   */
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
#include "aLAST.h"
#include "aLeafText.h"
#include "aStem.h"
#include "aCmd.tea"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local routine declarations
 */

static aBool sLAST_ParseReflexUnit(aLAST* pLAST, 
				   aASTNode** ppNewTree, 
				   aErr* pErr);

static aBool sLAST_ParseReflexDeclaration(aLAST* pLAST, 
				   	  aASTNode** ppNewTree, 
				   	  aErr* pErr);

static aBool sLAST_ParseModuleDeclaration(aLAST* pLAST, 
					  aASTNode** ppNewTree, 
					  aErr* pErr);

static aBool sLAST_ParseMessageDeclaration(aLAST* pLAST, 
					   aASTNode** ppNewTree, 
					   aErr* pErr);

static aBool sLAST_ParseVectorDeclaration(aLAST* pLAST, 
					  aASTNode** ppNewTree, 
					  aErr* pErr);

static aBool sLAST_ParseInlineDeclaration(aLAST* pLAST, 
					  aASTNode** ppNewTree, 
					  aErr* pErr);

static aBool sLAST_ParseModuleSpecifier(aLAST* pLAST, 
			              	aASTNode** ppNewTree, 
			                aErr* pErr);

static aBool sLAST_ParseMessageSpecifier(aLAST* pLAST, 
			              	 aASTNode** ppNewTree, 
			                 aErr* pErr);

static aBool sLAST_ParseVectorSpecifier(aLAST* pLAST, 
			              	aASTNode** ppNewTree, 
			                aErr* pErr);

static aBool sLAST_ParsePacketData(aLAST* pLAST,
		      	           aASTNode** ppNewTree,
		       	           aErr* pErr); 

static aBool sLAST_ParseMessageReferenceList(aLAST* pLAST,
		      	        	     aASTNode** ppNewTree,
		       	                     aErr* pErr);

static aBool sLAST_ParseMessageReference(aLAST* pLAST,
		      	        	 aASTNode** ppNewTree,
		       	                 aErr* pErr);

static aBool sLAST_ParseLModifyMessageReference(aLAST* pLAST, 
			                 aASTNode** ppNewTree, 
			                 aErr* pErr);

static aBool sLAST_ParseRModifyMessageReference(aLAST* pLAST, 
			                 aASTNode** ppNewTree, 
			                 aErr* pErr);

static aBool sLAST_ParseMessageModifyReference(aLAST* pLAST,
		      	        	       aASTNode** ppNewTree,
		       	                       aErr* pErr);

static aBool sLAST_ParseConstantValue(aLAST* pLAST,
		      	             aASTNode** ppNewTree,
		       	             aErr* pErr);

static aBool sLAST_ParseCastExpression(aLAST* pLAST,
		      	             aASTNode** ppNewTree,
		       	             aErr* pErr);

static aBool sLAST_ParsePrimaryExpression(aLAST* pLAST,
				          aASTNode** ppNewTree,
				          aErr* pErr);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * optimization prototypes
 */

static aErr sLOptVisit(aLAST* pLAST,
		       aASTNode* pNode);

static aErr sLOIReflexSpecifier(aLAST* pLAST, 
		    	        aASTNode* pReflexSpecifier);

static aErr sLOIPacketData(aLAST* pLAST, 
		    	   aASTNode* pPacketData);

static aErr sLOIMessageReference(aLAST* pLAST, 
		    	         aASTNode* pMessageReference);

static aErr sLOILModifyMessageReference(aLAST* pLAST, 
				       aASTNode* pLModifyMessageReference);

static aErr sLOIRModifyMessageReference(aLAST* pLAST, 
				        aASTNode* pRModifyMessageReference);

static aErr sLOIMessageModifyReference(aLAST* pLAST, 
				       aASTNode* pMessageModifyReference);

static aErr sLOIConstantValueProducer(aLAST* pLAST,
				     aASTNode* pNode,
				     aSFlags flags);

static aErr sLOIConstantValue(aLAST* pLAST, 
			     aASTNode* pConstantValue);

static aErr sLOICastExpression(aLAST* pLAST,
			       aASTNode* pCastExpression,
			       aSFlags flags);

static aErr sLOIPrimaryExpression(aLAST* pLAST, 
			          aASTNode* pConstantValue,
			          aSFlags flags);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * generation prototypes
 */

static aErr sLGenVisit(aLAST* pLAST,
		       aASTNode* pNode);

static aErr sLGenVisitModuleDeclaration(aLAST* pLAST,
					aASTNode* pNode);

static aErr sLGenVisitMessageDeclaration(aLAST* pLAST,
					 aASTNode* pNode);

static aErr sLGenVisitPacketData(aLAST* pLAST,
				 aASTNode* pNode,
				 char* data);

static aErr sLGenVisitMessageReferenceList(aLAST* pLAST,
				 	   aASTNode* pNode,
					   const int vectorIndex);

static aErr sLGenVisitMessageReference(aLAST* pLAST,
				       aASTNode* pReference,
				       const int vectorIndex,
				       const int refIndex);

static aErr sLGenVisitVectorDeclaration(aLAST* pLAST,
					aASTNode* pNode);

static aErr sLGenVisitInlineDeclaration(aLAST* pLAST,
					aASTNode* pNode);

static aErr sDumpReflexes(aLAST* pLAST);

static void sStringCatFmt(char* buffer,
			  unsigned char value);


static aReflexData* sGetReflexData(aLAST* pLAST);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sAST_ParseReflexUnit
 */

aBool sLAST_ParseReflexUnit(aLAST* pLAST, 
			    aASTNode** ppNewTree, 
			    aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree = NULL;

  *ppNewTree = NULL;

  while ((*pErr == aErrNone) 
         && (sLAST_ParseModuleDeclaration(pLAST, &pSubTree, pErr))) {
     
    /* build the parent node once */    
    if ((*pErr == aErrNone) &&
        (*ppNewTree == NULL)) {
      *pErr = aASTNode_Create((aAST*)pLAST, tReflexUnit, ppNewTree);
    }

    bMatched = aTrue;

    /* if there was a match, build up the node */
    if (*pErr == aErrNone) {
      aAssert(pSubTree);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
    }
    
    pSubTree = NULL;

  } /* while */

  /* we need at least one module declaration */  
  if ((*pErr == aErrNone) && (*ppNewTree == NULL)) {
    aToken* pToken = NULL;
    pLAST->nErrors++;
    aTokenizer_Next(pLAST->ioRef, pLAST->tokenizer, 
      		    &pToken, pErr);
    if (pLAST->errProc) {
      const char* data[2];
      unsigned int line = 0;
      unsigned int column = 0;
      aTokenInfo ti;

      data[0] = "unknown";
      data[1] = aLEAF_MODULE_EXPECTED;

      if (pToken) {
        aToken_GetInfo(aStreamLibRef(pLAST->errStream), 
        	       pToken, &ti, pErr);
        if (*pErr == aErrNone) {
          data[0] = ti.pSourceName;
          line = ti.nLine;
          column = ti.nColumn;
        }
      }
      if (*pErr == aErrNone)
        *pErr = pLAST->errProc(tkErrCompile, line, column, 
      			       2, data, pLAST->errProcRef);
      if (pToken)
        aTokenizer_PushBack(pLAST->ioRef, pLAST->tokenizer, 
      			    pToken, NULL);
    }
  }

  return bMatched;

} /* sLAST_ParseReflexUnit */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseReflexDeclaration
 */

aBool sLAST_ParseReflexDeclaration(aLAST* pLAST, 
			    	   aASTNode** ppNewTree, 
				   aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree = NULL;

  *ppNewTree = NULL;

  while ((*pErr == aErrNone) 
      && (sLAST_ParseMessageDeclaration(pLAST, &pSubTree, pErr)
          || sLAST_ParseVectorDeclaration(pLAST, &pSubTree, pErr)
          || sLAST_ParseInlineDeclaration(pLAST, &pSubTree, pErr)
         )) {

    /* build the parent node once */    
    if ((*pErr == aErrNone) &&
        (*ppNewTree == NULL)) {
      *pErr = aASTNode_Create((aAST*)pLAST, tReflexDeclaration, ppNewTree);
    }


    bMatched = aTrue;

    /* if there was a match, build up the node */
    if (*pErr == aErrNone) {
      aAssert(pSubTree);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
    }

    pSubTree = NULL;

  } /* while */

  return bMatched;

} /* sLAST_ParseReflexDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseModuleDeclaration
 */

aBool sLAST_ParseModuleDeclaration(aLAST* pLAST, 
			           aASTNode** ppNewTree, 
			           aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSpecifier = NULL;
  aASTNode* pReflexes = NULL;

  if ((*pErr == aErrNone)
        && sLAST_ParseModuleSpecifier(pLAST, &pSpecifier, pErr)
        && aAST_ParseSpecialChar((aAST*)pLAST, '{', aTrue, pErr)
        && sLAST_ParseReflexDeclaration(pLAST, &pReflexes, pErr)
        && aAST_ParseSpecialChar((aAST*)pLAST, '}', aTrue, pErr)) {
      bMatched = aTrue;

    if ((*pErr == aErrNone)
        && (*ppNewTree == NULL)) {
      *pErr = aASTNode_Create((aAST*)pLAST, 
      			      tModuleDeclaration, ppNewTree);
    }

    /* clean up the tokens either way */
    if ((bMatched == aTrue)
        && (*pErr == aErrNone)) {
      *pErr = aASTNode_AddChild(*ppNewTree, pSpecifier);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pReflexes);
    } else {
      if (*ppNewTree != NULL) {
        aASTNode_Destroy((aAST*)pLAST, *ppNewTree);
        *ppNewTree = NULL;
      }
    }
  }
  
  /* clean up when it failed */
  if ((*pErr == aErrNone)
      && (bMatched == aFalse)) {
    if (pSpecifier)
      aASTNode_Destroy((aAST*)pLAST, pSpecifier);
    if (pReflexes)
      aASTNode_Destroy((aAST*)pLAST, pReflexes);
  }

  return bMatched;

} /* sLAST_ParseModuleDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseMessageDeclaration
 */

aBool sLAST_ParseMessageDeclaration(aLAST* pLAST, 
			            aASTNode** ppNewTree, 
			            aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSpecifier = NULL;
  aASTNode* pData = NULL;

  if ((*pErr == aErrNone)
        && sLAST_ParseMessageSpecifier(pLAST, &pSpecifier, pErr)
        && aAST_ParseSpecialChar((aAST*)pLAST, '{', aTrue, pErr)
        && sLAST_ParsePacketData(pLAST, &pData, pErr)
        && aAST_ParseSpecialChar((aAST*)pLAST, '}', aTrue, pErr)) {
      bMatched = aTrue;

    if ((*pErr == aErrNone)
        && (*ppNewTree == NULL)) {
      *pErr = aASTNode_Create((aAST*)pLAST, 
      			      tMessageDeclaration, ppNewTree);
    }

    /* clean up the tokens either way */
    if ((bMatched == aTrue)
        && (*pErr == aErrNone)) {
      *pErr = aASTNode_AddChild(*ppNewTree, pSpecifier);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pData);
    } else {
      if (*ppNewTree != NULL) {
        aASTNode_Destroy((aAST*)pLAST, *ppNewTree);
        *ppNewTree = NULL;
      }
    }
  }
  
  /* clean up when it failed */
  if ((*pErr == aErrNone)
      && (bMatched == aFalse)) {
    if (pSpecifier)
      aASTNode_Destroy((aAST*)pLAST, pSpecifier);
    if (pData)
      aASTNode_Destroy((aAST*)pLAST, pData);
  }

  return bMatched;

} /* sLAST_ParseMessageDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseVectorDeclaration
 */

aBool sLAST_ParseVectorDeclaration(aLAST* pLAST, 
			           aASTNode** ppNewTree, 
			           aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSpecifier = NULL;
  aASTNode* pReferenceList = NULL;

  if ((*pErr == aErrNone)
        && sLAST_ParseVectorSpecifier(pLAST, &pSpecifier, pErr)
        && aAST_ParseSpecialChar((aAST*)pLAST, '{', aTrue, pErr)
        && sLAST_ParseMessageReferenceList(pLAST, &pReferenceList, pErr)
        && aAST_ParseSpecialChar((aAST*)pLAST, '}', aTrue, pErr)) {
      bMatched = aTrue;

    if ((*pErr == aErrNone)
        && (*ppNewTree == NULL)) {
      *pErr = aASTNode_Create((aAST*)pLAST, tVectorDeclaration, ppNewTree);
    }

    /* clean up the tokens either way */
    if ((bMatched == aTrue)
        && (*pErr == aErrNone)) {
      *pErr = aASTNode_AddChild(*ppNewTree, pSpecifier);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pReferenceList);
    } else {
      if (*ppNewTree != NULL) {
        aASTNode_Destroy((aAST*)pLAST, *ppNewTree);
        *ppNewTree = NULL;
      }
    }
  }

  /* clean up when it failed */
  if ((*pErr == aErrNone)
      && (bMatched == aFalse)) {
    if (pReferenceList != NULL)
      aASTNode_Destroy((aAST*)pLAST, pReferenceList);
    if (pSpecifier != NULL)
      aASTNode_Destroy((aAST*)pLAST, pSpecifier);
  }

  return bMatched;

} /* sLAST_ParseVectorDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseInlineDeclaration
 */

aBool sLAST_ParseInlineDeclaration(aLAST* pLAST, 
			           aASTNode** ppNewTree, 
			           aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pData = NULL;
  aSFlags flags = 0;
  aToken* pToken;

  if ((*pErr == aErrNone)
      && (!aTokenizer_Next(pLAST->ioRef, 
      			   pLAST->tokenizer, 
      			   &pToken, NULL))) {
    if (pToken->eType == tkIdentifier) {
      if (aStringCompare(pToken->v.identifier, aLR_PREFIX) == 0)
        flags |= fInlinePfx;
      else if (aStringCompare(pToken->v.identifier, aLR_SUFFIX) == 0)
        flags |= fInlineSfx;
    }

    if (flags != 0) {
       if (aAST_ParseSpecialChar((aAST*)pLAST, '{', aTrue, pErr)
           && sLAST_ParsePacketData(pLAST, &pData, pErr)
           && aAST_ParseSpecialChar((aAST*)pLAST, '}', aTrue, pErr)
          ) {

         *pErr = aASTNode_Create((aAST*)pLAST, 
      			        tInlineDeclaration, ppNewTree);
         if (*pErr == aErrNone) {
           bMatched = aTrue;
           (*ppNewTree)->pToken = pToken;
           (*ppNewTree)->flags |= flags;
           *pErr = aASTNode_AddChild(*ppNewTree, pData);
         }
       } else {
         flags = 0;
       }
    }
    
    /* push it back if it is not the right token */
    if ((*pErr != aErrNone) || (flags == 0)) {
      aTokenizer_PushBack(pLAST->ioRef, 
        		  pLAST->tokenizer, 
      			  pToken, 
      			  pErr);
      if (pData != NULL)
        aASTNode_Destroy((aAST*)pLAST, pData);
    }
  }

  return bMatched;

} /* sLAST_ParseInlineDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseModuleSpecifier
 */

aBool sLAST_ParseModuleSpecifier(aLAST* pLAST, 
			         aASTNode** ppNewTree, 
			         aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pIndex = NULL;

  /* check for the reserved start first */
  if ((*pErr == aErrNone)
      && aAST_ParseIdentifierToken((aAST*)pLAST, aLR_MODULE, aFalse, pErr)
      && aAST_ParseSpecialChar((aAST*)pLAST, '[', aTrue, pErr)
      && sLAST_ParseConstantValue(pLAST, &pIndex, pErr)
      && aAST_ParseSpecialChar((aAST*)pLAST, ']', aTrue, pErr)) {
    bMatched = aTrue;

    *pErr = aASTNode_Create((aAST*)pLAST, tModuleSpecifier, ppNewTree);

    /* clean up the tokens either way */
    if ((bMatched == aTrue)
        && (*pErr == aErrNone)) {
      *pErr = aASTNode_AddChild(*ppNewTree, pIndex);
    } else {
      if (*ppNewTree != NULL) {
        aASTNode_Destroy((aAST*)pLAST, *ppNewTree);
        *ppNewTree = NULL;
      }
      if (pIndex != NULL)
        aASTNode_Destroy((aAST*)pLAST, pIndex);
    }
  }

  return bMatched;

} /* sLAST_ParseModuleSpecifier */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseMessageSpecifier
 */

aBool sLAST_ParseMessageSpecifier(aLAST* pLAST, 
			          aASTNode** ppNewTree, 
			          aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pIndex = NULL;

  /* check for the reserved start first */
  if ((*pErr == aErrNone)
      && aAST_ParseIdentifierToken((aAST*)pLAST, aLR_MESSAGE, aFalse, pErr)
      && aAST_ParseSpecialChar((aAST*)pLAST, '[', aTrue, pErr)
      && sLAST_ParseConstantValue(pLAST, &pIndex, pErr)
      && aAST_ParseSpecialChar((aAST*)pLAST, ']', aTrue, pErr)) {
    bMatched = aTrue;

    *pErr = aASTNode_Create((aAST*)pLAST, tMessageSpecifier, ppNewTree);

    /* clean up the tokens either way */
    if ((bMatched == aTrue)
        && (*pErr == aErrNone)) {
      *pErr = aASTNode_AddChild(*ppNewTree, pIndex);
    } else {
      if (*ppNewTree != NULL) {
        aASTNode_Destroy((aAST*)pLAST, *ppNewTree);
        *ppNewTree = NULL;
      }
      if (pIndex != NULL)
        aASTNode_Destroy((aAST*)pLAST, pIndex);
    }
  }

  return bMatched;

} /* sLAST_ParseMessageSpecifier */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseVectorSpecifier
 */

aBool sLAST_ParseVectorSpecifier(aLAST* pLAST, 
			         aASTNode** ppNewTree, 
			         aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pIndex = NULL;

  /* check for the reserved start first */
  if ((*pErr == aErrNone)
      && aAST_ParseIdentifierToken((aAST*)pLAST, aLR_VECTOR, aFalse, pErr)
      && aAST_ParseSpecialChar((aAST*)pLAST, '[', aTrue, pErr)
      && sLAST_ParseConstantValue(pLAST, &pIndex, pErr)
      && aAST_ParseSpecialChar((aAST*)pLAST, ']', aTrue, pErr)) {
    bMatched = aTrue;

    *pErr = aASTNode_Create((aAST*)pLAST, tVectorSpecifier, ppNewTree);

    /* clean up the tokens either way */
    if ((bMatched == aTrue)
        && (*pErr == aErrNone)) {
      *pErr = aASTNode_AddChild(*ppNewTree, pIndex);
    } else {
      if (*ppNewTree != NULL) {
        aASTNode_Destroy((aAST*)pLAST, *ppNewTree);
        *ppNewTree = NULL;
      }
      if (pIndex != NULL)
        aASTNode_Destroy((aAST*)pLAST, pIndex);
    }
  }

  return bMatched;

} /* sLAST_ParseVectorSpecifier */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParsePacketData
 */

aBool sLAST_ParsePacketData(aLAST* pLAST,
		      	    aASTNode** ppNewTree,
		       	    aErr* pErr)
{
  aBool bMatched = aFalse;
  aBool bExpressionDone = aFalse;
  aBool bPreceedingComma = aFalse;
  aASTNode* pSubExpression = NULL;

  *ppNewTree = NULL;

  /* start by creating the empty list */
  if (*pErr == aErrNone) {
    *pErr = aASTNode_Create((aAST*)pLAST, tPacketData, ppNewTree);
    if (*pErr == aErrNone)
      bMatched = aTrue;
  }

  /* gobble up each of the parameters */
  while ((*pErr == aErrNone)
         && (bExpressionDone == aFalse)) {
    
    if (sLAST_ParseConstantValue(pLAST, &pSubExpression, pErr)) {

      /* add the new expression to the list */
      if (*pErr == aErrNone) {
        aAssert(*ppNewTree);
        *pErr = aASTNode_AddChild(*ppNewTree, pSubExpression);
        if (*pErr == aErrNone) {
          pSubExpression = NULL;
        }
      }

      /* now check for the comma separating the sub-expressions */
      if (*pErr == aErrNone) {
        if (aAST_ParseSpecialChar((aAST*)pLAST, ',', aFalse, pErr))
          bPreceedingComma = aTrue;
        else
          bExpressionDone = aTrue;
      }

    } else if (bPreceedingComma) {
      *pErr = aAST_OutputNextTokenError((aAST*)pLAST, 
      					aLEAF_CONSTANT_BYTE_EXPECTED);
      bExpressionDone = aTrue;
      bMatched = aFalse;
    }
  }

  if (bMatched == aFalse) {
    aASTNode_Destroy((aAST*)pLAST,
    		     *ppNewTree);
    *ppNewTree = NULL;
  }

  return bMatched;

} /* sLAST_ParsePacketData */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseMessageReferenceList
 */

aBool sLAST_ParseMessageReferenceList(aLAST* pLAST,
		      	   	      aASTNode** ppNewTree,
		       	    	      aErr* pErr)
{
  aBool bMatched = aFalse;
  aBool bListDone = aFalse;
  aASTNode* pSubList = NULL;

  *ppNewTree = NULL;

  /* gobble up each of the parameters */
  while ((*pErr == aErrNone)
         && (bListDone == aFalse)
         && sLAST_ParseMessageReference(pLAST, &pSubList, pErr)) {

    /* since we got a valid expression, create a list to work with */
    if ((*pErr == aErrNone)
        && (*ppNewTree == NULL)) {
      *pErr = aASTNode_Create((aAST*)pLAST, 
      			      tMessageReferenceList, ppNewTree);
    }

    /* now add the new expression to the list */
    if (*pErr == aErrNone) {
      aAssert(*ppNewTree);
      *pErr = aASTNode_AddChild(*ppNewTree, pSubList);
      if (*pErr == aErrNone) {
        pSubList = NULL;
        bMatched = aTrue;
      }
    }
    
    /* now check for the comma separating the sub-expressions */
    if ((*pErr == aErrNone)
        && !aAST_ParseSpecialChar((aAST*)pLAST, ',', aFalse, pErr))
      bListDone = aTrue;
  }

  return bMatched;

} /* sLAST_ParseMessageReferenceList */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseMessageReference
 */

aBool sLAST_ParseMessageReference(aLAST* pLAST, 
			          aASTNode** ppNewTree, 
			          aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pReference = NULL;

  /* a message index is required */
  if ((*pErr == aErrNone)
      && (sLAST_ParseRModifyMessageReference(pLAST,
      					        &pReference, pErr)
      	  || sLAST_ParseLModifyMessageReference(pLAST,
      					     &pReference, pErr)
      	  || sLAST_ParseConstantValue(pLAST, &pReference, pErr)
      	  )) {
    bMatched = aTrue;

    *pErr = aASTNode_Create((aAST*)pLAST, 
    			    tMessageReference, ppNewTree);

    /* clean up the tokens either way */
    if ((bMatched == aTrue)
        && (*pErr == aErrNone)) {
      *pErr = aASTNode_AddChild(*ppNewTree, pReference);
    } else {
      if (*ppNewTree != NULL) {
        aASTNode_Destroy((aAST*)pLAST, *ppNewTree);
        *ppNewTree = NULL;
      }
      if (pReference != NULL)
        aASTNode_Destroy((aAST*)pLAST, pReference);
    }
  }

  return bMatched;

} /* sLAST_ParseMessageReference */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseLModifyMessageReference
 */

aBool sLAST_ParseLModifyMessageReference(aLAST* pLAST, 
			                 aASTNode** ppNewTree, 
			                 aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pReference = NULL;
  aASTNode* pType = NULL;
  aSFlags flags = 0;
  aToken* pToken = NULL;

  /* a message index is required first */
  if ((*pErr == aErrNone)
      && sLAST_ParseMessageModifyReference(pLAST,
      					   &pReference, pErr)) {
    if (!aTokenizer_Next(pLAST->ioRef, 
      			pLAST->tokenizer, 
      			&pToken, NULL)) {
      if (pToken->eType == tkSpecial) {
        char errMsg[100];
        switch (pToken->v.special) {
        case '-':
          flags |= fLModMinus;
          break;
        case '=':
          flags |= fLModEqual;
          break;
        default:
          /* report the error and clean up */
          errMsg[0] = pToken->v.special;
          errMsg[1] = 0;
          aStringCat(errMsg, aLEAF_ILLEGAL_OPCODE);
          *pErr = aAST_OutputError((aAST*)pLAST, NULL, errMsg);
        } /* switch */
        
        /* destroy the token either way */
        aTokenizer_Dispose(pLAST->ioRef,
        		   pLAST->tokenizer,
        		   pToken,
        		   pErr);
        if (flags) {
          if (aAST_ParseTypeSpecifier((aAST*)pLAST,
          			      &pType,
          			      pErr)) {
            bMatched = aTrue;
            *pErr = aASTNode_Create((aAST*)pLAST, 
    			            tLModifyMessageReference, 
    			            ppNewTree);
            if (*pErr == aErrNone) {
              (*ppNewTree)->flags |= flags;
              *pErr = aASTNode_AddChild(*ppNewTree, pReference);
              if (*pErr == aErrNone)
                *pErr = aASTNode_AddChild(*ppNewTree, pType);              
            }
          }
        }
      }
    }

    if (!bMatched) {
      *pErr = aAST_OutputError((aAST*)pLAST, 
      			       pReference, aLEAF_MODOP_EXPECTED);
    }
  }

  return bMatched;

} /* sLAST_ParseLModifyMessageReference */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseRModifyMessageReference
 */

aBool sLAST_ParseRModifyMessageReference(aLAST* pLAST, 
			                 aASTNode** ppNewTree, 
			                 aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pReference = NULL;
  aASTNode* pType = NULL;
  aSFlags flags = 0;
  aToken* pToken = NULL;

  /* a message index is required first */
  if ((*pErr == aErrNone)
      && aAST_ParseTypeSpecifier((aAST*)pLAST,
      				 &pType, pErr)) {
    if (!aTokenizer_Next(pLAST->ioRef, 
      			pLAST->tokenizer, 
      			&pToken, NULL)) {
      if (pToken->eType == tkSpecial) {
        char errMsg[100];
        switch (pToken->v.special) {
        case '+':
          flags |= fRModPlus;
          break;
        case '-':
          flags |= fRModMinus;
          break;
        case '*':
          flags |= fRModMult;
          break;
        case '>':
          if (aAST_ParseSpecialChar((aAST*)pLAST, '>', aTrue, pErr))
            flags |= fRModRShift;
          break;
        default:
          /* report the error and clean up */
          errMsg[0] = pToken->v.special;
          errMsg[1] = 0;
          aStringCat(errMsg, aLEAF_ILLEGAL_OPCODE);
          *pErr = aAST_OutputError((aAST*)pLAST, NULL, errMsg);
        } /* switch */
        
        /* destroy the token either way */
        aTokenizer_Dispose(pLAST->ioRef,
        		   pLAST->tokenizer,
        		   pToken,
        		   pErr);
        if (flags
            && sLAST_ParseMessageModifyReference(pLAST,
      					         &pReference, 
      					         pErr)) {
          bMatched = aTrue;
          *pErr = aASTNode_Create((aAST*)pLAST, 
    			          tRModifyMessageReference, 
    			          ppNewTree);
          if (*pErr == aErrNone) {
            (*ppNewTree)->flags |= flags;
            *pErr = aASTNode_AddChild(*ppNewTree, pType);
            if (*pErr == aErrNone) {
              *pErr = aASTNode_AddChild(*ppNewTree, pReference);
            }
          }
        }
      }
    }

    if (!bMatched) {
      *pErr = aAST_OutputError((aAST*)pLAST, 
      			       pReference, aLEAF_MODOP_EXPECTED);
    }
  }

  return bMatched;

} /* sLAST_ParseRModifyMessageReference */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseMessageMofifyReference
 */

aBool sLAST_ParseMessageModifyReference(aLAST* pLAST, 
			                aASTNode** ppNewTree, 
			                aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pIndex = NULL;
  aASTNode* pInsert = NULL;

  /* check for the reserved start first */
  if ((*pErr == aErrNone)
      && sLAST_ParseConstantValue(pLAST, &pIndex, pErr)) {

    if (aAST_ParseSpecialChar((aAST*)pLAST, '[', aFalse, pErr)
        && sLAST_ParseConstantValue(pLAST, &pInsert, pErr)
        && aAST_ParseSpecialChar((aAST*)pLAST, ']', aTrue, pErr)) {
      bMatched = aTrue;

      *pErr = aASTNode_Create((aAST*)pLAST, 
    			      tMessageModifyReference, ppNewTree);

      /* clean up the tokens either way */
      if ((bMatched == aTrue)
          && (*pErr == aErrNone)) {
        *pErr = aASTNode_AddChild(*ppNewTree, pIndex);
        if (*pErr == aErrNone)
          *pErr = aASTNode_AddChild(*ppNewTree, pInsert);
      } else {
        if (*ppNewTree != NULL) {
          aASTNode_Destroy((aAST*)pLAST, *ppNewTree);
          *ppNewTree = NULL;
        }
        if (pIndex != NULL)
          aASTNode_Destroy((aAST*)pLAST, pIndex);
        if (pInsert != NULL)
          aASTNode_Destroy((aAST*)pLAST, pInsert);
      }
    } else {
      /* push back the constant byte as we failed */
      aASTNode_Unwind((aAST*)pLAST, pIndex);
    }
  }

  return bMatched;

} /* sLAST_ParseMessageModifyReference */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseConstantValue
 */

aBool sLAST_ParseConstantValue(aLAST* pLAST,
			      aASTNode** ppNewTree,
			      aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;

  if ((*pErr == aErrNone)
      && sLAST_ParseCastExpression(pLAST, &pSubTree, pErr)) {
    *pErr = aASTNode_Create((aAST*)pLAST, tConstantValue, ppNewTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
    if (*pErr == aErrNone)
      bMatched = aTrue;
  }

  if ((*pErr == aErrNone)
      && (bMatched == aFalse)) {
    aToken* pToken;
    if (!aTokenizer_Next(((aAST*)pLAST)->ioRef, 
    			 ((aAST*)pLAST)->tokenizer, 
      			 &pToken, NULL)) {

      /* identifier means something wasn't defined */
      if (pToken->eType == tkIdentifier) {
        aASTNode* pTemp;
        *pErr = aASTNode_Create((aAST*)pLAST, tConstantValue, &pTemp);
        if (*pErr == aErrNone) {
          char msg[50];
          pTemp->pToken = pToken;
          aStringCopy(msg, pToken->v.identifier);
          aStringCat(msg, aLEAF_UNDEFINED_IDENT);
          *pErr = aAST_OutputError((aAST*)pLAST,
        		           pTemp,
        		           msg);
          if (*pErr == aErrNone)
            aASTNode_Destroy((aAST*)pLAST, pTemp); 
        }
      } else {
        /* push it back if it is not the right token */
        aTokenizer_PushBack(pLAST->ioRef, 
        		    pLAST->tokenizer, 
      			    pToken, 
      			    pErr);
      }

    }
  }

  return bMatched;

} /* sLAST_ParseConstantValue */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParseCastExpression
 */

aBool sLAST_ParseCastExpression(aLAST* pLAST,
			        aASTNode** ppNewTree,
			        aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;
  aToken* pToken;
  
  *ppNewTree = NULL;

  /* see if it is a cast expression */
  if (!aTokenizer_Next(pLAST->ioRef, pLAST->tokenizer, &pToken, NULL)) {
    if ((pToken->eType == tkSpecial)
        && (pToken->v.special == '(')
        && aAST_ParseDeclarationSpecifier((aAST*)pLAST, &pSubTree, pErr)
        && aAST_ParseSpecialChar((aAST*)pLAST, ')', aTrue, pErr)) {
      aTokenizer_Dispose(pLAST->ioRef, pLAST->tokenizer, pToken, pErr);
      *pErr = aASTNode_Create((aAST*)pLAST, tCastExpression, ppNewTree);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
    } else {
      aTokenizer_PushBack(pLAST->ioRef, pLAST->tokenizer, pToken, pErr);
    }
  }

  if (*pErr == aErrNone) {
    if (sLAST_ParsePrimaryExpression(pLAST, &pSubTree, pErr)) {
      if (*ppNewTree == NULL)
        *ppNewTree = pSubTree;
      else
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);        
      bMatched = aTrue;
    }
  }

  return bMatched;

} /* sLAST_ParseCastExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLAST_ParsePrimaryExpression
 */

aBool sLAST_ParsePrimaryExpression(aLAST* pLAST,
				   aASTNode** ppNewTree,
				   aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;

  if ((*pErr == aErrNone)
      && (aAST_ParseConstant((aAST*)pLAST, &pSubTree, pErr)
          || (aAST_ParseSpecialChar((aAST*)pLAST, '(', aFalse, pErr)
              && sLAST_ParseConstantValue(pLAST, &pSubTree, pErr)
              && aAST_ParseSpecialChar((aAST*)pLAST, ')', aTrue, pErr)))) {
      *pErr = aASTNode_Create((aAST*)pLAST, tPrimaryExpression, ppNewTree);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
      if (*pErr == aErrNone)
        bMatched = aTrue;
    }

  return bMatched;

} /* sLAST_ParsePrimaryExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLOptVisit
 */

aErr sLOptVisit(aLAST* pLAST,
		aASTNode* pNode)
{
  aErr lastErr = aErrNone;

  switch (pNode->eType) {

  case tModuleSpecifier:
  case tMessageSpecifier:
  case tVectorSpecifier:
    lastErr = sLOIReflexSpecifier(pLAST, pNode);
    break;

  case tMessageReference:
    lastErr = sLOIMessageReference(pLAST, pNode);
    break;

  case tPacketData:
    lastErr = sLOIPacketData(pLAST, pNode);
    break;

  default:
    if ((lastErr == aErrNone)
        && pNode->pChildren)
      lastErr = sLOptVisit(pLAST, pNode->pChildren);
    if ((lastErr == aErrNone)
        && pNode->pNext)
      lastErr = sLOptVisit(pLAST, pNode->pNext);
    break;

  } /* switch */

  return lastErr;

} /* sLOptVisit */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLOIReflexSpecifier
 */

aErr sLOIReflexSpecifier(aLAST* pLAST, 
		         aASTNode* pReflexSpecifier)
{
  aErr lastErr = aErrNone;
  aASTNode* pIndex;

  aAssert(pLAST);

  /* walk through and purify all the external declarations */
  pIndex = pReflexSpecifier->pChildren;
  aAssert(pIndex);

  /* the index of the module, message or vector is the first child */
  lastErr = sLOIConstantValue(pLAST, pIndex);

  /* then, walk the list for the rest of the children */  
  if (lastErr == aErrNone)
    lastErr = sLOptVisit(pLAST, pReflexSpecifier->pNext);

  return lastErr;

} /* sLOIReflexSpecifier */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLOIPacketData
 */

aErr sLOIPacketData(aLAST* pLAST, 
		    aASTNode* pPacketData)
{
  aErr lastErr = aErrNone;
  aASTNode* pValue;

  aAssert(pLAST);
  aAssert(pPacketData->eType == tPacketData);

  /* walk through and purify all the packet data bytes */
  pValue = pPacketData->pChildren;
  while ((pValue != NULL) && (lastErr == aErrNone)) {
    lastErr = sLOIConstantValue(pLAST, pValue);
    pValue = pValue->pNext;
  }

  return lastErr;

} /* sLOIPacketData */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLOIMessageReference
 */

aErr sLOIMessageReference(aLAST* pLAST, 
		          aASTNode* pMessageReference)
{
  aErr lastErr = aErrNone;
  aASTNode* pChild;
  
  aAssert(pLAST);
  aAssert(pMessageReference->eType == tMessageReference);

  /* walk through and purify all the external declarations */
  pChild = pMessageReference->pChildren;
  if (pChild != NULL) {
    switch (pChild->eType) {

    case tLModifyMessageReference:
      lastErr = sLOILModifyMessageReference(pLAST, pChild);
      break;

    case tRModifyMessageReference:
      lastErr = sLOIRModifyMessageReference(pLAST, pChild);
      break;

    case tConstantValue:
      lastErr = sLOIConstantValue(pLAST, pChild);
      break;
    
    default:
      aAssert(0);
      break;

    } /* switch */
  } /* if pChild */

  if ((lastErr == aErrNone)
      && (pMessageReference->pNext != NULL))
    lastErr = sLOIMessageReference(pLAST, pMessageReference->pNext);

  return lastErr;

} /* sLOIMessageReference */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLOILModifyMessageReference
 */

aErr sLOILModifyMessageReference(aLAST* pLAST, 
				 aASTNode* pLModifyMessageReference)
{
  aErr lastErr = aErrNone;
  aASTNode* pReference;
  
  aAssert(pLAST);
  aAssert(pLModifyMessageReference->eType == tLModifyMessageReference);

  /* walk through and purify all the external declarations */
  pReference = pLModifyMessageReference->pChildren;

  lastErr = sLOIMessageModifyReference(pLAST, pReference);

  return lastErr;

} /* sLOILModifyMessageReference */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLOIRModifyMessageReference
 */

aErr sLOIRModifyMessageReference(aLAST* pLAST, 
				 aASTNode* pRModifyMessageReference)
{
  aErr lastErr = aErrNone;
  aASTNode* pReference;
  
  aAssert(pLAST);
  aAssert(pRModifyMessageReference->eType == tRModifyMessageReference);

  /* walk through and purify all the external declarations */
  pReference = pRModifyMessageReference->pChildren;
  aAssert(pReference);
  pReference = pReference->pNext;

  lastErr = sLOIMessageModifyReference(pLAST, pReference);

  return lastErr;

} /* sLOIRModifyMessageReference */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLOIMessageModifyReference
 */

aErr sLOIMessageModifyReference(aLAST* pLAST, 
		          	aASTNode* pMessageModifyReference)
{
  aErr lastErr = aErrNone;
  aASTNode* pIndex;
  aASTNode* pOffset;
  
  aAssert(pLAST);
  aAssert(pMessageModifyReference->eType == tMessageModifyReference);

  /* walk through and purify all the external declarations */
  pIndex = pMessageModifyReference->pChildren;
  
  lastErr = sLOIConstantValue(pLAST, pIndex);

  if (lastErr == aErrNone) {
    pOffset = pIndex->pNext;
    lastErr = sLOIConstantValue(pLAST, pOffset);
  }

  return lastErr;

} /* sLOIMessageModifyReference */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLOIConstantValueProducer
 */
 
aErr sLOIConstantValueProducer(aLAST* pLAST,
			      aASTNode* pNode,
			      aSFlags flags)
{
  aErr lastErr = aErrNone;

  switch (pNode->eType) {

  case tCastExpression:
    lastErr = sLOICastExpression(pLAST, pNode, flags);
    break;

  case tPrimaryExpression:
    lastErr = sLOIPrimaryExpression(pLAST, pNode, flags);
    break;

  case tConstant:
    lastErr = aAST_OptConstant((aAST*)pLAST, pNode, flags);
    break;

  default:
    aAssert(0); /* should we get here? */
    break;

  } /* switch */
  
  return lastErr;

} /* sLOIConstantValueProducer */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLOIConstantValue
 */
 
aErr sLOIConstantValue(aLAST* pLAST,
		      aASTNode* pConstantValue)
{
  aErr lastErr = aErrNone;
  aASTNode* pChild;

  aAssert(pConstantValue);

  pChild = pConstantValue->pChildren;
  
  if (pChild) {
    lastErr = sLOIConstantValueProducer(pLAST, pChild, 
    					fConstant | fGetType);

    if (lastErr == aErrNone)
      lastErr = aAST_PushUpFlags(pConstantValue, pChild);
  }

  return lastErr;

} /* sLOIConstantValue */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLOICastExpression 
 */

aErr sLOICastExpression(aLAST* pLAST,
			aASTNode* pCastExpression,
			aSFlags flags)
{
  aErr lastErr = aErrNone;
  aASTNode* pType;
  aASTNode* pChild;
  
  aAssert(pCastExpression);
  aAssert(pCastExpression->eType == tCastExpression);

  pCastExpression->flags |= flags;

  pType = pCastExpression->pChildren;
  aAssert(pType);
  aAssert(pType->eType == tDeclarationSpecifier);
  lastErr = aAST_OptDeclarationSpecifier((aAST*)pLAST, pType);

  if ((lastErr == aErrNone)
      && (pCastExpression->flags & fGetType))
    pCastExpression->flags |= pType->flags & (fTypeMask | fUnsigned);

  if (lastErr == aErrNone) {
    pChild = pType->pNext;
    aAssert(pChild);

    /* purify the sub-type and get its type */
    lastErr = sLOIPrimaryExpression(pLAST, pChild, 
  			          (flags & ~fTypeMask) | fGetType);
  }

  /* just do the cast here if constant and there is actually 
   * a conversion */
  if ((lastErr == aErrNone)
      && (pChild->flags & fConstant)) {
    pCastExpression->flags |= fConstant;
    if ((pChild->flags & fByte)
        && (pType->flags & fShort))
      pCastExpression->v.shortVal = pChild->v.byteVal;
    else if ((pChild->flags & fShort)
             && (pType->flags & fByte))
      pCastExpression->v.byteVal = (tBYTE)pChild->v.shortVal;
    else if ((pChild->flags & fShort)
             && (pType->flags & fShort))
      pCastExpression->v.shortVal = pChild->v.shortVal;
    else if ((pChild->flags & fByte)
             && (pType->flags & fByte))
      pCastExpression->v.byteVal = pChild->v.byteVal;
    else
      lastErr = pLAST->outputErrProc(pLAST, pCastExpression, 
      				   aLEAF_INVALID_CAST);
  }

  return lastErr;

} /* sLOICastExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLOIPrimaryExpression
 */
 
aErr sLOIPrimaryExpression(aLAST* pLAST,
		           aASTNode* pPrimaryExpression,
		           aSFlags flags)
{
  aErr lastErr = aErrNone;
  aASTNode* pChildren;

  aAssert(pPrimaryExpression);
  aAssert(pPrimaryExpression->eType == tPrimaryExpression);
  
  pPrimaryExpression->flags |= flags;

  pChildren = pPrimaryExpression->pChildren;
  aAssert(pChildren);

  switch (pChildren->eType) {

  case tConstant:
    lastErr = aAST_OptConstant((aAST*)pLAST, pChildren, flags);
    break;

  default:
    aAssert(0); /* should we get here ? */
    break;

  } /* switch */
  
  if (lastErr == aErrNone)
    lastErr = aAST_PushUpFlags(pPrimaryExpression, pChildren);
  
  return lastErr;

} /* sLOIPrimaryExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLGenVisit
 */

aErr sLGenVisit(aLAST* pLAST,
		aASTNode* pNode)
{
  aErr lastErr = aErrNone;

  switch (pNode->eType) {
 
  case tModuleDeclaration:
    sLGenVisitModuleDeclaration(pLAST, pNode);
    break;
 
  case tMessageDeclaration:
    sLGenVisitMessageDeclaration(pLAST, pNode);
    break;

  case tVectorDeclaration:
    sLGenVisitVectorDeclaration(pLAST, pNode);
    break;

  case tInlineDeclaration:
    sLGenVisitInlineDeclaration(pLAST, pNode);
    break;

  default:
    if ((lastErr == aErrNone)
        && pNode->pChildren)
      lastErr = sLGenVisit(pLAST, pNode->pChildren);
    if ((lastErr == aErrNone)
        && pNode->pNext)
      lastErr = sLGenVisit(pLAST, pNode->pNext);
    break;

  } /* switch */

  return lastErr;

} /* sLGenVisit */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLGenVisitModuleDeclaration
 */

aErr sLGenVisitModuleDeclaration(aLAST* pLAST,
				 aASTNode* pNode)
{
  aErr lastErr = aErrNone;
  tBYTE i;
  aASTNode* pSpecifier;

  aAssert(pLAST);
  aAssert(pNode);
  aAssert(pNode->eType == tModuleDeclaration);

  switch (pLAST->nPass) {

  case 0: /* range checking */
    pSpecifier = pNode->pChildren;
    aAssert(pSpecifier); /* points to message-specifier */
    aAssert(pSpecifier->pChildren);
    i = pSpecifier->pChildren->v.byteVal;
    if (((unsigned char)i < 2) || ((unsigned char)i > 254))
      lastErr = aAST_OutputError((aAST*)pLAST, 
      				 pSpecifier->pChildren, 
      				 aLEAF_MODULE_OUTOFRANGE);
    else if (i % 2)
      lastErr = aAST_OutputError((aAST*)pLAST, 
      				 pSpecifier->pChildren, 
      				 aLEAF_MODULE_ODD);
    else
      pNode->t.module.address = i;
 
    /* fall through */

  case 1: /* configuration generation */
  
    /* set the current module */
    pNode->t.module.pNext = pLAST->pCurModule;
    pLAST->pCurModule = pNode;
    
    /* visit the contained reflex declarations */
    if ((lastErr == aErrNone)
        && pNode->pChildren)
      lastErr = sLGenVisit(pLAST, pNode->pChildren);
    if ((lastErr == aErrNone)
        && pNode->pNext)
      lastErr = sLGenVisit(pLAST, pNode->pNext);
      
    /* pop off the current module definition */
    aAssert(pLAST->pCurModule == pNode);
    pLAST->pCurModule = pNode->t.module.pNext;    
    break;

  case 2:
    /* we don't do anything here */
    break;

  default: /* shouldn't get here */
    aAssert(0);
    break;

  } /* switch */

  return lastErr;

} /* sLGenVisitModuleDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLGenVisitMessageDeclaration
 */

aErr sLGenVisitMessageDeclaration(aLAST* pLAST,
				  aASTNode* pNode)
{
  aErr lastErr = aErrNone;
  int i;
  aASTNode* pSpecifier;
  aASTNode* pPacketData;

  aAssert(pLAST);
  aAssert(pNode);
  aAssert(pNode->eType == tMessageDeclaration);

  switch (pLAST->nPass) {

  case 0: /* range checking */
    pSpecifier = pNode->pChildren;
    aAssert(pSpecifier); /* points to message-specifier */
    aAssert(pSpecifier->pChildren);
    i = pSpecifier->pChildren->v.byteVal;
    if ((i < 0) || (i >= aNUMMESSAGES)) {
      lastErr = aAST_OutputError((aAST*)pLAST, 
      				 pSpecifier->pChildren, 
      				 aLEAF_MSGINDEX_OUTOFRANGE);
    } else {
      aReflexData* pModule = sGetReflexData(pLAST);
      if (pModule->message[i].bUsed) {
      lastErr = aAST_OutputError((aAST*)pLAST, 
      				 pSpecifier->pChildren, 
      				 aLEAF_MSGINDEX_USED);
    } else {
        pModule->nMessages++;
        pModule->message[i].bUsed = aTrue;
    }
    pPacketData = pSpecifier->pNext;
    lastErr = sLGenVisitPacketData(pLAST, pPacketData, 
    				     pModule->message[i].data);
    }
    
    /* fallthrough */

  case 1: /* configuration generation */
    if ((lastErr == aErrNone) && (pNode->pNext))
      lastErr = sLGenVisit(pLAST, pNode->pNext);
    break;

  default: /* shouldn't get here */
    break;

  } /* switch */

  return lastErr;

} /* sLGenVisitMessageDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLGenVisitPacketData
 */

aErr sLGenVisitPacketData(aLAST* pLAST,
			  aASTNode* pNode,
			  char* data)
{
  aErr lastErr = aErrNone;
  aASTNode* pValue;
  int count;

  aAssert(pLAST);
  aAssert(pNode);
  aAssert(pNode->eType == tPacketData);
  aAssert(data);

  switch (pLAST->nPass) {

  case 0:
  case 2:

    /* message must at least have address */
    pValue = pNode->pChildren;
    count = 0;
    if (pValue == NULL) {
      lastErr = aAST_OutputError((aAST*)pLAST, 
      				 pNode,
      				 aLEAF_MSGSIZE_UNDERRANGE);
    }

    /* first data value must be an address byte */
    else {
      if (!pValue->flags & fByte) {
        lastErr = aAST_OutputError((aAST*)pLAST, 
      				   pValue,
      				   aLEAF_ADDRESS_RANGE_ERROR);
      } else {
        data[count] = pValue->v.byteVal;
      }
      count += 2; /* skip over the length storage */
      pValue = pValue->pNext;
    }

    /* now, accumulate the rest of the message data */    
    while ((pValue != NULL)
           && (lastErr == aErrNone)) {
      if (pValue->flags & fByte) {
        data[count++] = pValue->v.byteVal;
      } else {
        aUtil_StoreShort(&data[count],
        		pValue->v.shortVal);
        count += sizeof(tSHORT);
      }

      /* check to see if we have exceeded the space
       * which includes the address and length bytes */
      if (count > aSTEMMAXPACKETBYTES + 2) {
        lastErr = aAST_OutputError((aAST*)pLAST, 
      				   pValue,
      				   aLEAF_MSGSIZE_OVERRANGE);
        break;
      }
      pValue = pValue->pNext;
    }
    data[1] = (char)(count - 2);
    break;

  case 1: /* configuration generation */
    break;

  default: /* shouldn't get here */
    break;

  } /* switch */

  return lastErr;

} /* sLGenVisitPacketData */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLGenVisitVectorDeclaration
 */

aErr sLGenVisitVectorDeclaration(aLAST* pLAST,
				 aASTNode* pNode)
{
  aErr lastErr = aErrNone;
  aASTNode* pSpecifier;
  aASTNode* pMessageReferenceList;
  int i;
  
  aAssert(pLAST);
  aAssert(pNode);
  aAssert(pNode->eType == tVectorDeclaration);

  switch (pLAST->nPass) {

  case 0: /* range checking */
    break;

  case 1: /* configuration generation */
    pSpecifier = pNode->pChildren;
    aAssert(pSpecifier); /* points to vector-specifier */
    aAssert(pSpecifier->pChildren);
    i = pSpecifier->pChildren->v.byteVal;
    if ((i < 0) || (i >= aNUMVECTORS)) {
      lastErr = aAST_OutputError((aAST*)pLAST, 
      				 pSpecifier->pChildren, 
      				 aLEAF_VECTORINDEX_OUTOFRANGE);
    } else {
      aReflexData* pModule = sGetReflexData(pLAST);
      if (pModule->vector[i].bUsed) {
      lastErr = aAST_OutputError((aAST*)pLAST, 
      				 pSpecifier->pChildren, 
      				 aLEAF_VECTORINDEX_USED);
    } else {
        pModule->nVectors++;
        pModule->vector[i].bUsed = aTrue;
    }
    pMessageReferenceList = pSpecifier->pNext;
    lastErr = sLGenVisitMessageReferenceList(pLAST, 
    					     pMessageReferenceList, 
    					     i);
    }

    /* remaining vectors follow so visit if present */
    if ((lastErr == aErrNone) && (pNode->pNext))
      lastErr = sLGenVisit(pLAST, pNode->pNext);
    break;

  default: /* shouldn't get here */
    aAssert(0);
    break;

  } /* switch */

  return lastErr;

} /* sLGenVisitVectorDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLGenVisitInlineDeclaration
 */

aErr sLGenVisitInlineDeclaration(aLAST* pLAST,
				 aASTNode* pNode)
{
  aErr lastErr = aErrNone;
  char buf[60];
  char data[aSTEMMAXPACKETBYTES + 2];
  int left;
  int i;
  
  aAssert(pLAST);
  aAssert(pNode);
  aAssert(pNode->eType == tInlineDeclaration);

  if ((((pLAST->nPass == 0) && (pNode->flags & fInlinePfx))
      || ((pLAST->nPass == 2) && (pNode->flags & fInlineSfx)))) {

    /* extract the data */
    lastErr = sLGenVisitPacketData(pLAST, pNode->pChildren, data);
  
    /* build a comment */
    if (pNode->flags & fInlinePfx)
      aStringCopy(buf, "// prefix");
    else
      aStringCopy(buf, "// suffix");
    aStream_WriteLine(pLAST->ioRef,
      		      pLAST->outputStream,
      		      buf,
      		      &lastErr);

    /* left is packet length plus address and size */
    left = data[1] + 1;
    buf[0] = 0; /* start with null string */
    for (i = 0; i <= left; i++) {
      if (i != 1)
        sStringCatFmt(buf, (unsigned char)data[i]);
    }
    
    aStream_WriteLine(pLAST->ioRef,
      		      pLAST->outputStream,
      		      buf,
      		      &lastErr);
  }

  return lastErr;

} /* sLGenVisitInlineDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLGenVisitMessageReferenceList
 */

aErr sLGenVisitMessageReferenceList(aLAST* pLAST,
				    aASTNode* pNode,
				    const int vectorIndex)
{
  aErr lastErr = aErrNone;
  aASTNode* pReference;
  char count;
  aReflexData* pModule;

  aAssert(pLAST);
  aAssert(pNode);
  aAssert(vectorIndex >= 0);
  aAssert(vectorIndex < aNUMMESSAGES);

  switch (pLAST->nPass) {

  case 0: /* range checking */
    break;

  case 1: /* configuration generation */
    pReference = pNode->pChildren;
    count = 0;
    if (pReference == NULL) {
      lastErr = aAST_OutputError((aAST*)pLAST, 
      				 pNode,
      				 aLEAF_MSGSIZE_UNDERRANGE);
    }
    while ((pReference != NULL)
           && (lastErr == aErrNone)) {
      /* check to see if we have exceeded the space
       * which includes the address and length bytes */
      if (count > aNUMVECTORREFS) {
        lastErr = aAST_OutputError((aAST*)pLAST, 
      				   pReference,
      				   aLEAF_VECTORSIZE_OVERRANGE);
        break;
      }
      lastErr = sLGenVisitMessageReference(pLAST,
      					   pReference,
      					   vectorIndex,
      					   count++);
      pReference = pReference->pNext;
    }
    /* set the last bit */
    pModule = sGetReflexData(pLAST);
    pModule->vector[vectorIndex].data[count - 1][0] |= (char)0x80;
    break;

  default: /* shouldn't get here */
    break;

  } /* switch */

  return lastErr;

} /* sLGenVisitMessageReferenceList */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLGenVisitMessageReference
 */

aErr sLGenVisitMessageReference(aLAST* pLAST,
				aASTNode* pReference,
				const int vectorIndex,
				const int refIndex)
{
  aErr lastErr = aErrNone;
  aASTNode* pChild;
  aASTNode* pIndex = NULL;
  aASTNode* pOffset = NULL;
  int message;
  char op = 0;
  aReflexData* pModule;

  aAssert(pLAST);
  aAssert(pReference);
  aAssert(pReference->eType == tMessageReference);
  aAssert(vectorIndex >= 0);
  aAssert(vectorIndex < aNUMMESSAGES);

  switch (pLAST->nPass) {

  case 0: /* range checking */
    break;

  case 1: /* configuration generation */
    pChild = pReference->pChildren;
    switch (pChild->eType) {

    case tConstantValue:
      pIndex = pChild;
      break;

    case tLModifyMessageReference:

      /* set up the operation */
      switch (pChild->flags & 0xFFFF0000) {
      case fLModEqual:
        op = 7;
        break;
      case fLModMinus:
        op = 2;
        break;
      } /* switch */

      pChild = pChild->pChildren;
      aAssert(pChild);
      aAssert(pChild->eType == tMessageModifyReference);
      pIndex = pChild->pChildren;
      pOffset = pIndex->pNext;
      break;

    case tRModifyMessageReference:

      /* set up the operation */
      switch (pChild->flags & 0xFFFF0000) {
      case fRModPlus:
        op = 1;
        break;
      case fRModMinus:
        op = 3;
        break;
      case fRModRShift:
        op = 4;
        break;
      case fRModMult:
        if (pChild->pChildren->flags & fByte)
          op = 5;
        else
          op = 6;
        break;
      } /* switch */

      pChild = pChild->pChildren;
      pChild = pChild->pNext;
      aAssert(pChild);
      aAssert(pChild->eType == tMessageModifyReference);
      pIndex = pChild->pChildren;
      pOffset = pIndex->pNext;
      break;

    default:
      aAssert(0); /* should we get here ? */
      break;

    } /* switch */

    message = pIndex->v.byteVal;
    pModule = sGetReflexData(pLAST);
    if ((message < 0) || (message >= aNUMMESSAGES)) {
      lastErr = aAST_OutputError((aAST*)pLAST, 
      				 pIndex, 
      				 aLEAF_MSGINDEX_OUTOFRANGE);
    } else if (pModule->message[message].bUsed != aTrue) {
      lastErr = aAST_OutputError((aAST*)pLAST, 
      				 pIndex, 
      				 aLEAF_MSGINDEX_NOTUSED);
    } else {
      pModule->vector[vectorIndex].data[refIndex][0] |= (char)message;
    }

    /* set up the modification offset */
    if (op != 0) {
      char offset;
 
      aAssert(pOffset);
      offset = pOffset->v.byteVal;
      if (((signed char)offset < 0) || 
      	  (offset > pModule->message[message].data[1] + 1)) {
        lastErr = aAST_OutputError((aAST*)pLAST, 
      				   pOffset, 
      				   aLEAF_MSGOFFSET_RANGE);
      } else {
        pModule->vector[vectorIndex].data[refIndex][1] |= offset;
      }
    }

    /* mark vector as used */
    pModule->vector[vectorIndex].bUsed = aTrue;

    /* add in the operation bits */
    pModule->vector[vectorIndex].data[refIndex][1] |= (char)(op << 4);

    /* add in the enable bit */
    pModule->vector[vectorIndex].data[refIndex][1] |= (char)0x80;
    break;

  default: /* shouldn't get here */
    break;

  } /* switch */

  return lastErr;

} /* sLGenVisitMessageReference */


void sStringCatFmt(char* buffer,
		   unsigned char value)
{
  char num[6];
#if 0
  char temp;
  value = value & 0xFF;
  num[0] = '0';
  num[1] = 'x';
  temp = (char)(value / 16);
  num[2] = (char)((temp < 10) ? ('0' + temp) : ('A' + temp - 10));
  temp = (char)(value % 16);
  num[3] = (char)((temp < 10) ? ('0' + temp) : ('A' + temp - 10));
  num[4] = ' ';
  num[5] = 0;
#else
  aStringFromInt(num, value);
  aStringCat(num, " ");
#endif
  aStringCat(buffer, num);

} /* sStringCatFmt */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sDumpReflexes
 */

aErr sDumpReflexes(aLAST* pLAST)
{
  aErr lastErr = aErrNone;
  char buffer[100];
  char num[5];
  int left;
  char lineBytes;
  int index;
  int i;
  int m;
  unsigned char dest;
  aReflexData* pModule;
  aBool bDone;

  /* dump for each module */
  for (m = 0; (m < 126) && (lastErr == aErrNone); m++) {

    pModule = pLAST->modules[m];
    if (!pModule)
      continue;
    
    /* ignore any modules we didn't involve */
    if (!pModule->nMessages && !pModule->nVectors)
      continue;

    dest = (unsigned char)((m + 1) * 2);

    /* build a comment */
    aStringFromInt(num, dest);
    aStream_WriteLine(pLAST->ioRef,
      		      pLAST->outputStream, "", &lastErr);
    aStringCopy(buffer, "// module ");
    aStringCat(buffer, num);
    aStringCat(buffer, " (");
    aStringFromInt(num, pModule->nMessages);
    aStringCat(buffer, num);
    aStringCat(buffer, " messages, ");
    aStringFromInt(num, pModule->nVectors);
    aStringCat(buffer, num);
    aStringCat(buffer, " vectors)");
    aStream_WriteLine(pLAST->ioRef,
      		      pLAST->outputStream,
      		      buffer,
      		      &lastErr);
    aStream_WriteLine(pLAST->ioRef,
      		      pLAST->outputStream, "", &lastErr);
  
  /* dump the messages */
  for (i = 0; (i < aNUMMESSAGES) && (lastErr == aErrNone); i++) {
      if (pModule->message[i].bUsed) {

      /* build a comment */
      aStringFromInt(num, i);
      aStringCopy(buffer, "// message ");
      aStringCat(buffer, num);
      aStream_WriteLine(pLAST->ioRef,
      			pLAST->outputStream,
      			buffer,
      			&lastErr);

      /* left is packet length plus address and size */
        left = (char)(pModule->message[i].data[1] + 2);
        index = 0;

        bDone = aFalse;
        while (!bDone) {
          buffer[0] = 0; /* start with null string */
          sStringCatFmt(buffer, dest);
        sStringCatFmt(buffer, cmdMSG_WR);
        if (left <= 5) {
          sStringCatFmt(buffer, (unsigned char)(0x80 + index));
          sStringCatFmt(buffer, (unsigned char)i);
          lineBytes = 4;
          bDone = aTrue;
        } else {
          sStringCatFmt(buffer, 0);
          lineBytes = 3;
        }
        while ((lineBytes++ < aSTEMMAXPACKETBYTES + 1)
               && (left-- > 0)) {
          sStringCatFmt(buffer, 
          		(unsigned char)pModule->message[i].data[index++]);
          }
          aStream_WriteLine(pLAST->ioRef,
      			  pLAST->outputStream,
      			  buffer,
      			  &lastErr);
        }
      }
    } /* for each message */

  /* dump the vectors */
  for (i = 0; (i < aNUMVECTORS) && (lastErr == aErrNone); i++) {
      if (pModule->vector[i].bUsed) {

      /* build a comment */
      aStringFromInt(num, i);
      aStringCopy(buffer, "// vector ");
      aStringCat(buffer, num);
      aStream_WriteLine(pLAST->ioRef,
      			pLAST->outputStream,
      			buffer,
      			&lastErr);

      /* left is number of total vector bytes */
      for (left = 0; left < aNUMVECTORREFS; left++) {
          if (pModule->vector[i].data[left][0] & 0x80)
          break;
      }
      left = (char)((left + 1) * 2);

      index = 0;

      bDone = aFalse;
      while (!bDone) {
        buffer[0] = 0; /* start with null string */
          sStringCatFmt(buffer, dest);
        sStringCatFmt(buffer, cmdVEC_WR);
        if (left <= 5) {
          sStringCatFmt(buffer, (unsigned char)(0x80 + (index * 2)));
          sStringCatFmt(buffer, (unsigned char)i);
          lineBytes = 4;
          bDone = aTrue;
        } else {
          sStringCatFmt(buffer, (unsigned char)(index * 2));
          lineBytes = 3;
        }
        while ((lineBytes < aSTEMMAXPACKETBYTES + 1)
               && (left > 0)) {
          sStringCatFmt(buffer, 
          		(unsigned char)pModule->vector[i].data[index][0]);
            sStringCatFmt(buffer, 
          		(unsigned char)pModule->vector[i].data[index++][1]);
          lineBytes += 2;
          left -= 2;
        }
        aStream_WriteLine(pLAST->ioRef,
      			  pLAST->outputStream,
      			  buffer,
      			  &lastErr);
      }
    }

    } /* for each vector */
 
  } /* for each module */

  return lastErr;

} /* sDumpReflexes */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sGetReflexData
 */

aReflexData* sGetReflexData(aLAST* pLAST)
{
  int module;
  int index;
  
  
  if (pLAST->pCurModule)
    module = pLAST->pCurModule->t.module.address;
  else
    module = 2;
  
  index = module / 2 - 1;

  aAssert(pLAST);
  aAssert(index >= 0);
  aAssert(index < 125);

  if (!pLAST->modules[index]) {
    pLAST->modules[index] = aMemAlloc(sizeof(aReflexData));
    aAssert(pLAST->modules[index]);
    aBZero(pLAST->modules[index], sizeof(aReflexData));
  }

  return pLAST->modules[index];

} /* sGetReflexData */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLAST_Create
 */

aErr aLAST_Create(aIOLib ioRef,
		  aLAST** ppLAST,
		  aTokenErrProc errProc,
		  void* errProcRef)
{
  aErr lastErr = aErrNone;
  aLAST* pLAST;

  aAssert(ppLAST);
  *ppLAST = NULL;

  pLAST = (aLAST*)aMemAlloc(sizeof(aLAST));
  if (pLAST == NULL)
    lastErr = aErrMemory;

  if (lastErr == aErrNone) {
    aBZero(pLAST, sizeof(aLAST));
    pLAST->errProc = errProc;
    pLAST->errProcRef = errProcRef;
    pLAST->outputErrProc = aAST_OutputError;
    pLAST->ioRef = ioRef;
  }

  if (lastErr == aErrNone)
    aMemPool_Create(ioRef, sizeof(aASTNode),
    		    aASTNODEPOOLSIZE, &pLAST->ASTNodePool,
    		    &lastErr);

  if (lastErr == aErrNone) {
    *ppLAST = pLAST;
  }

  return lastErr;

} /* aLAST_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLAST_Parse
 */

aErr aLAST_Parse(aLAST* pLAST,
		 aTokenizerRef tokenizerRef,
		 aStreamRef errStream)
{
  aErr lastErr = aErrNone;

  aAssert(pLAST);
  aAssert(pLAST->ASTNodePool);

  /* set up the LAST */
  pLAST->tokenizer = tokenizerRef;
  pLAST->errStream = errStream;

  /* do the parse */
  if (lastErr == aErrNone)
    sLAST_ParseReflexUnit(pLAST, &pLAST->pTree, &lastErr);

  return lastErr;

} /* aLAST_Parse */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLAST_Optimize
 */

aErr aLAST_Optimize(aLAST* pLAST,
		    aStreamRef errStream)
{
  aErr lastErr = aErrNone;
  
  aAssert(pLAST);

  /* do the optimization (constant folding) */
  if ((lastErr == aErrNone)
      && (pLAST->nErrors == 0)
      && (pLAST->pTree))
    lastErr = sLOptVisit(pLAST, pLAST->pTree);

  return lastErr;

} /* aLAST_Optimize */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLAST_Generate
 */

aErr aLAST_Generate(aLAST* pLAST,
		    aStreamRef outputStream)
{
  aErr lastErr = aErrNone;
  
  aAssert(pLAST);
  aAssert(outputStream);

  pLAST->outputStream = outputStream;

  if ((lastErr == aErrNone)
      && (pLAST->nErrors == 0)) {
    pLAST->nPass = 0;
    lastErr = sLGenVisit(pLAST, pLAST->pTree);
  }

  if ((lastErr == aErrNone)
      && (pLAST->nErrors == 0)) {
    pLAST->nPass = 1;
    lastErr = sLGenVisit(pLAST, pLAST->pTree);
  }

  if ((lastErr == aErrNone)
      && (pLAST->nErrors == 0)) {
    lastErr = sDumpReflexes(pLAST);
  }

  if ((lastErr == aErrNone)
      && (pLAST->nErrors == 0)) {
    pLAST->nPass = 2;
    lastErr = sLGenVisit(pLAST, pLAST->pTree);
  }

  return lastErr;

} /* aLAST_Generate */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLAST_Destroy
 */

aErr aLAST_Destroy(aLAST* pLAST)
{
  aErr lastErr = aErrNone;
  int i;

  aAssert(pLAST);

  if (pLAST->pTree)
    aASTNode_Destroy((aAST*)pLAST, pLAST->pTree);

  if (pLAST->ASTNodePool != NULL) {
    aMemPool_Destroy(pLAST->ioRef, pLAST->ASTNodePool, &lastErr);
    pLAST->ASTNodePool = NULL;
  }

  /* clean up the module reflex storage */  
  for (i = 0; i < 126; i++) {
    if (pLAST->modules[i]) {
      aMemFree(pLAST->modules[i]);
      pLAST->modules[i] = NULL;
    }
  }

  aMemFree((aMemPtr)pLAST);

  return lastErr;

} /* aLAST_Destroy */

