/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSAST.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of an SAST (annotated syntax tree)  */
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
#include "aSAST.h"
#include "aSteepOpt.h"
#include "aSteepText.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local routine declarations
 */

static aBool sSAST_ParseTranslationUnit(aSAST* pSAST, 
				       aASTNode** ppNewTree, 
				       aErr* pErr);
static aBool sSAST_ParseExternalDeclaration(aSAST* pSAST, 
					   aASTNode** ppNewTree, 
					   aErr* pErr);
static aBool sSAST_ParseFunctionDefinition(aSAST* pSAST, 
					  aASTNode** ppNewTree, 
					  aErr* pErr);
static aBool sSAST_ParseInitDeclaratorList(aSAST* pSAST,
					  aASTNode** ppNewTree,
					  aErr* pErr);
static aBool sSAST_ParseInitDeclarator(aSAST* pSAST,
				      aASTNode** ppNewTree,
				      aErr* pErr);
static aBool sSAST_ParseDeclarator(aSAST* pSAST, 
				  aASTNode** ppNewTree,
				  const aBool bRequired,
				  aErr* pErr);
static aBool sSAST_ParseParameterList(aSAST* pSAST,
				     aASTNode** ppNewTree,
				     aErr* pErr);
static aBool sSAST_ParseParameterDeclaration(aSAST* pSAST,
					    aASTNode** ppNewTree,
					    aErr* pErr);
static aBool sSAST_ParseCompoundStatement(aSAST* pSAST, 
					 aASTNode** ppNewTree,
					 aErr* pErr);
static aBool sSAST_ParseDeclarationList(aSAST* pSAST,
				       aASTNode** ppNewTree,
				       aErr* pErr);
static aBool sSAST_ParseDeclaration(aSAST* pSAST,
				   aASTNode** ppNewTree,
				   aErr* pErr);
static aBool sSAST_ParseStatementList(aSAST* pSAST,
				     aASTNode** ppNewTree,
				     aErr* pErr);
static aBool sSAST_ParseStatement(aSAST* pSAST,
				 aASTNode** ppNewTree,
				 aErr* pErr);
static aBool sSAST_ParseAsmStatement(aSAST* pSAST,
				    aASTNode** ppNewTree,
				    aErr* pErr);
static aBool sSAST_ParseJumpStatement(aSAST* pSAST,
				     aASTNode** ppNewTree,
				     aErr* pErr);
static aBool sSAST_ParseAsmList(aSAST* pSAST,
			       aASTNode** ppNewTree,
			       aErr* pErr);
static aBool sSAST_ParseOpcode(aSAST* pSAST,
			      aASTNode** ppNewTree,
			      aErr* pErr);
static aBool sSAST_ParseExpression(aSAST* pSAST,
			          aASTNode** ppNewTree,
			          aErr* pErr);
static aBool sSAST_ParseString(aSAST* pSAST,
			      aASTNode** ppNewTree,
			      aErr* pErr);
static aBool sSAST_ParseIdentifier(aSAST* pSAST,
			          aASTNode** ppNewTree,
			          aErr* pErr);
static aBool sSAST_ParseLabeledStatement(aSAST* pSAST,
			     	        aASTNode** ppNewTree,
			     	        aErr* pErr);
static aBool sSAST_ParseExpressionStatement(aSAST* pSAST,
			     	           aASTNode** ppNewTree,
			     	           aErr* pErr);
static aBool sSAST_ParseAssignmentExpression(aSAST* pSAST,
					    aASTNode** ppNewTree,
					    aErr* pErr);
static aBool sSAST_ParseUnaryExpression(aSAST* pSAST,
				       aASTNode** ppNewTree,
				       aErr* pErr);
static aBool sSAST_ParseAssignmentOperator(aSAST* pSAST,
					  aASTNode** ppNewTree,
					  aErr* pErr);
static aBool sSAST_ParsePostFixExpression(aSAST* pSAST,
					 aASTNode** ppNewTree,
					 aErr* pErr);
static aBool sSAST_ParseArgumentExpressionList(aSAST* pSAST,
					      aASTNode** ppNewTree,
					      aErr* pErr);
static aBool sSAST_ParsePrimaryExpression(aSAST* pSAST,
				  	 aASTNode** ppNewTree,
				  	 aErr* pErr);
static aBool sSAST_ParseSelectionStatement(aSAST* pSAST,
				  	  aASTNode** ppNewTree,
				  	  aErr* pErr);
static aBool sSAST_ParseLoopStatement(aSAST* pSAST,
				     aASTNode** ppNewTree,
				     aErr* pErr);
static aBool sSAST_ParseInitializer(aSAST* pSAST,
				   aASTNode** ppNewTree,
				   aErr* pErr);
static aBool sSAST_ParseUnaryOperator(aSAST* pSAST,
				     aASTNode** ppNewTree,
				     aErr* pErr);
static aBool sSAST_ParseCastExpression(aSAST* pSAST,
				      aASTNode** ppNewTree,
				      aErr* pErr);
static aBool sSAST_ParseConditionalExpression(aSAST* pSAST,
				             aASTNode** ppNewTree,
				             aErr* pErr);
static aBool sSAST_ParseLogicalORExpression(aSAST* pSAST,
				           aASTNode** ppNewTree,
				           aErr* pErr);
static aBool sSAST_ParseLogicalANDExpression(aSAST* pSAST,
				            aASTNode** ppNewTree,
				            aErr* pErr);
static aBool sSAST_ParseEqualityExpression(aSAST* pSAST,
					  aASTNode** ppNewTree,
					  aErr* pErr);
static aBool sSAST_ParseRelationalExpression(aSAST* pSAST,
					    aASTNode** ppNewTree,
					    aErr* pErr);
static aBool sSAST_ParseShiftExpression(aSAST* pSAST,
				       aASTNode** ppNewTree,
				       aErr* pErr);
static aSFlags sSAST_AdditiveOperator(aToken* pToken);
static aBool sSAST_ParseAdditiveExpression(aSAST* pSAST,
					  aASTNode** ppNewTree,
					  aErr* pErr);
static aSFlags sSAST_MultiplicativeOperator(aToken* pToken);
static aBool sSAST_ParseMultiplicativeExpression(aSAST* pSAST,
					  	aASTNode** ppNewTree,
					  	aErr* pErr);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * generic handling for left-to-right evals
 */

typedef aBool (*pL2RProducerProc)(aSAST* pSAST,
				  aASTNode** ppNewTree,
				  aErr* pErr);
typedef aSFlags (*pL2RCombinerProc)(aToken* pToken);
static aBool sL2REval(aSAST* pSAST,
	              pL2RProducerProc subexpression,
		      pL2RCombinerProc combiner,
		      aASTNode** ppNewTree,
		      aErr* pErr);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseTranslationUnit
 */

aBool sSAST_ParseTranslationUnit(aSAST* pSAST, 
				aASTNode** ppNewTree, 
				aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree = NULL;

  *ppNewTree = NULL;

  while ((*pErr == aErrNone) 
         && sSAST_ParseExternalDeclaration(pSAST, &pSubTree, pErr)) {
    /* build the parent node once */    
    if ((*pErr == aErrNone) &&
        (*ppNewTree == NULL)) {
      *pErr = aASTNode_Create((aAST*)pSAST, tTranslationUnit, 
      			      ppNewTree);
    }

    bMatched = aTrue;

    /* if there was a match, build up the node */
    if (*pErr == aErrNone) {
      aAssert(pSubTree);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
    }
  } /* while */

  return bMatched;

} /* sSAST_ParseTranslationUnit */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseExternalDeclaration
 */

aBool sSAST_ParseExternalDeclaration(aSAST* pSAST, 
				     aASTNode** ppNewTree,
				     aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree = NULL;

  if ((*pErr == aErrNone) 
      && (sSAST_ParseFunctionDefinition(pSAST, &pSubTree, pErr)
          || sSAST_ParseDeclaration(pSAST, &pSubTree, pErr))) {
    bMatched = aTrue;
  }

  /* these are our only choices. Error if not the end of tokens */
  if ((*pErr == aErrNone) && !bMatched) {
    aToken* pToken = NULL;
    aASTNode* pTemp = NULL;
    if (!aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
    			 &pToken, NULL)) {
      *pErr = aASTNode_Create((aAST*)pSAST, tString, &pTemp);
      pTemp->pToken = pToken;
    }
    if (pToken && (*pErr == aErrNone))
      *pErr = aAST_OutputError(pSAST, pTemp, 
      			       aSE_ROUT_OR_DECL_EXPECTED);
    if (pTemp) {
      pTemp->pToken = NULL;
      if (*pErr == aErrNone)
        aASTNode_Destroy((aAST*)pSAST, pTemp);
    }

    if ((*pErr == aErrNone) && pToken) {
      aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, 
      			  pToken, pErr);
    }
  }

  /* if there was a match, build up the node */
  if ((*pErr == aErrNone) && bMatched) {
    *pErr = aASTNode_Create((aAST*)pSAST, tExternalDeclaration, 
    			    ppNewTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
  }

  return bMatched;

} /* sSAST_ParseExternalDeclaration */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseFunctionDefinition
 */

aBool sSAST_ParseFunctionDefinition(aSAST* pSAST, 
				   aASTNode** ppNewTree,
				   aErr* pErr)
{
  aBool bMatched = aFalse;
  aBool bFoundParams = aFalse;
  aASTNode* pDeclarationSpecifier = NULL; 
  aASTNode* pDeclarator = NULL;
  aASTNode* pCompoundStatement = NULL;

  /* optional declaration specifiers */
  if (*pErr == aErrNone)
    aAST_ParseDeclarationSpecifier((aAST*)pSAST, &pDeclarationSpecifier, pErr);

  /* required declarator */
  if (*pErr == aErrNone)
    sSAST_ParseDeclarator(pSAST, &pDeclarator, aTrue, pErr);

  /* check to see if the declarator has the required parameter list */
  if (pDeclarator) {
    aASTNode* pChild = pDeclarator->pChildren;
    while (pChild && !bMatched && pChild->eType != tParameterList)
    	pChild = pChild->pNext;
    if (pChild && pChild->eType == tParameterList)
      bFoundParams = aTrue;
  }

  /* and the compound statement */
  if ((*pErr == aErrNone) 
      && pDeclarator
      && bFoundParams
      && sSAST_ParseCompoundStatement(pSAST, &pCompoundStatement, pErr)) {
    bMatched = aTrue;
  }

  /* if there was a match, build up the node */
  if ((*pErr == aErrNone)
      && (bMatched == aTrue)) {
      
    /* we matched so build and populate the node */
    *pErr = aASTNode_Create((aAST*)pSAST, tFunctionDefinition, ppNewTree);

    /* add a default type specifier if none as present */
    if ((*pErr == aErrNone)
        && (pDeclarationSpecifier == NULL)) {
      *pErr = aASTNode_Create((aAST*)pSAST, tDeclarationSpecifier, 
      			      &pDeclarationSpecifier);
      /* add in the type specifier below the decl spec */
      if (*pErr == aErrNone) {
        aASTNode* pTypeSpec;
        *pErr = aASTNode_Create((aAST*)pSAST, tTypeSpecifier, &pTypeSpec);  
        if (*pErr == aErrNone) {
          pTypeSpec->flags |= fShort;
          *pErr = aASTNode_AddChild(pDeclarationSpecifier, 
          			    pTypeSpec);
        }
      }
    }
    if (*pErr == aErrNone) {
      aAssert(pDeclarationSpecifier);
      *pErr = aASTNode_AddChild(*ppNewTree, pDeclarationSpecifier);
    }

    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pDeclarator);

    if (*pErr == aErrNone) {
      aASTNode* pList;

      /* skip over the optional declaration list if present */
      pList = pCompoundStatement->pChildren;
      if ((pList != NULL)
          && (pList->eType == tDeclarationList))
        pList = pList->pNext;

      /* build a statement list if none were present */
      if (pList == NULL) {
        *pErr = aASTNode_Create((aAST*)pSAST, tStatementList, &pList);
        
        /* now add the statement list */
        if (*pErr == aErrNone)
          *pErr = aASTNode_AddChild(pCompoundStatement, pList);
      }

      /* now, make sure there is a return statement at the 
       * end */
      if (*pErr == aErrNone) {
        aASTNode* pTemp;
        aASTNode* pLast = NULL;
        pTemp = pList->pChildren;
        while (pTemp) {
          pLast = pTemp;
          pTemp = pTemp->pNext;
        }

        /* install a return value if not found */
        if (!pLast
            || (pLast->pChildren->eType != tJumpStatement)
            || (!(pLast->pChildren->flags & fJumpReturn))) {
          aASTNode* pReturn;
          aASTNode* pStatement;
          *pErr = aASTNode_Create((aAST*)pSAST, tStatement, &pStatement);
          if (*pErr == aErrNone) {
            *pErr = aASTNode_Create((aAST*)pSAST, tJumpStatement, &pReturn);
            pReturn->flags |= fJumpReturn;
            if (*pErr == aErrNone)
              *pErr = aASTNode_AddChild(pStatement, pReturn);
          }
          if (*pErr == aErrNone)
            *pErr = aASTNode_AddChild(pList, pStatement);
        }
      }
    
      /* add in the compound statement */
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pCompoundStatement);
    }
  }

  /* clean up the nodes if we failed to match */
  if (bMatched == aFalse) {
    if (pDeclarator)
      aASTNode_Unwind((aAST*)pSAST, pDeclarator);
    if (pDeclarationSpecifier)
      aASTNode_Unwind((aAST*)pSAST, pDeclarationSpecifier);
  }

  return bMatched;

} /* sSAST_ParseFunctionDefinition */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseInitDeclaratorList
 */

aBool sSAST_ParseInitDeclaratorList(aSAST* pSAST,
			       	   aASTNode** ppNewTree,
			       	   aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pInitDeclarator = NULL;

  /* gobble up each of the statements */
  while ((*pErr == aErrNone)
         && sSAST_ParseInitDeclarator(pSAST, &pInitDeclarator, pErr)) {

    /* since we got a valid declaration, create a list to work with */
    if ((*ppNewTree) == NULL)
      *pErr = aASTNode_Create((aAST*)pSAST, tInitDeclaratorList, ppNewTree);

    /* now add the new statement to the list */
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pInitDeclarator);

    if (*pErr == aErrNone)
      bMatched = aTrue;
    
    /* stop if we don't find a comma */
    if ((*pErr == aErrNone)
        && (aAST_ParseSpecialChar((aAST*)pSAST, ',', aFalse, pErr) == aFalse))
      break;
  }

  return bMatched;

} /* sSAST_ParseInitDeclaratorList */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseInitDeclarator
 */

aBool sSAST_ParseInitDeclarator(aSAST* pSAST,
			       aASTNode** ppNewTree,
			       aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pDeclarator = NULL;

  /* gobble up each of the statements */
  if ((*pErr == aErrNone)
      && sSAST_ParseDeclarator(pSAST, &pDeclarator, aFalse, pErr)) {
    aASTNode* pInitializer;

    bMatched = aTrue;

    /* build the init declarator node with the declarator in it */
    *pErr = aASTNode_Create((aAST*)pSAST, tInitDeclarator, ppNewTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pDeclarator);

    /* now check for optional initializer */
    if ((*pErr == aErrNone) 
        && aAST_ParseSpecialChar((aAST*)pSAST, '=', aFalse, pErr)
        && sSAST_ParseInitializer(pSAST, &pInitializer, pErr)) {
      *pErr = aASTNode_AddChild(*ppNewTree, pInitializer);
    }
  }

  return bMatched;

} /* sSAST_ParseInitDeclaratorList */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseDeclarator
 */

aBool sSAST_ParseDeclarator(aSAST* pSAST, 
			   aASTNode** ppNewTree,
			   const aBool bRequired,
			   aErr* pErr)
{
  aBool bMatched = aFalse;
  aBool bMatchedParamList = aFalse;
  aBool bOpenParen = aFalse;
  aASTNode* pIdentifier = NULL;
  aASTNode* pParameterList = NULL;
  aToken* pToken = NULL;

  /* all declarators start with an identifier 
   * followed by an optional parameter list
   */
  if (*pErr == aErrNone) {

    /* scan for the identifier */
    if (!aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
    			 &pToken, NULL)) {
      if (pToken->eType == tkIdentifier) {
        *pErr = aASTNode_Create((aAST*)pSAST, tIdentifier, &pIdentifier);
        if (*pErr == aErrNone) {
          pIdentifier->pToken = pToken;
        }        
      } else {
        /* if required, show and error here */
        if (bRequired) {
          aASTNode* pTemp;
          char errMsg[100];
          char badTypeName[32];
	  aTokenInfo ti;
          aStringCopySafe(errMsg, 100, aTE_IDENTIFIER_EXPECTED);	  
	  aToken_GetInfo(pSAST->ioRef, pToken, &ti, NULL);
	  aStringCopySafe(badTypeName, 32, ti.pTypeName);
          aStringCatSafe(errMsg, 100, badTypeName);
          *pErr = aASTNode_Create((aAST*)pSAST, tString, &pTemp);
          pTemp->pToken = pToken;
          if (*pErr == aErrNone)
            *pErr = aAST_OutputError(pSAST, pTemp, errMsg);
          pTemp->pToken = NULL;
          if (*pErr == aErrNone)
            aASTNode_Destroy((aAST*)pSAST, pTemp);
        }

        /* if no identifier was available, just clean up */
        aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
      }
    }
  }

  /* now check for a parameter list open paren */
  if ((*pErr == aErrNone)
      && (pIdentifier != NULL)
      && aAST_ParseSpecialChar((aAST*)pSAST, '(', aFalse, pErr)) {
      bOpenParen = aTrue;
  }

  /* now the parameter contents */
  if ((*pErr == aErrNone)
      && (bOpenParen == aTrue)) {
    sSAST_ParseParameterList(pSAST, &pParameterList, pErr);
  }
  
  /* now ensure the close paren */
  if ((*pErr == aErrNone)
      && (pParameterList != NULL)
      && aAST_ParseSpecialChar((aAST*)pSAST, ')', aTrue, pErr)) {
    bMatchedParamList = aTrue;
  }
  
  /* finally, build up the final node based on whether we matched */
  if (*pErr == aErrNone) {
    if (pIdentifier != NULL) {
      bMatched = aTrue;
      *pErr = aASTNode_Create((aAST*)pSAST, tDeclarator, ppNewTree);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pIdentifier);
      if ((*pErr == aErrNone)
          && (bMatchedParamList == aTrue)) {
        *pErr = aASTNode_AddChild(*ppNewTree, pParameterList);
      }
    } else {
      if (pParameterList != NULL)
        aASTNode_Destroy((aAST*)pSAST, pParameterList);
    }
  }

  return bMatched;

} /* sSAST_ParseDeclarator */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseParameterList
 */

aBool sSAST_ParseParameterList(aSAST* pSAST,
			      aASTNode** ppNewTree,
			      aErr* pErr)
{
  aBool bMatched = aFalse;
  aBool bListDone = aFalse;
  aASTNode* pParameter = NULL;

  *ppNewTree = NULL;

  /* gobble up each of the parameters */
  while ((*pErr == aErrNone)
         && (bListDone == aFalse)
         && sSAST_ParseParameterDeclaration(pSAST, &pParameter, pErr)) {

    /* since we got a valid parameter, create a list to work with */
    if ((*pErr == aErrNone)
        && (*ppNewTree == NULL)) {
      *pErr = aASTNode_Create((aAST*)pSAST, tParameterList, ppNewTree);
    }

    /* now add the new parameter to the list */
    if (*pErr == aErrNone) {
      aAssert(*ppNewTree);
      *pErr = aASTNode_AddChild(*ppNewTree, pParameter);
      if (*pErr == aErrNone) {
        pParameter = NULL;
        bMatched = aTrue;
      }
    }

    /* now check for the comma separating the parameters */
    if ((*pErr == aErrNone)
        && !aAST_ParseSpecialChar((aAST*)pSAST, ',', aFalse, pErr)) {
      bListDone = aTrue;
    }
  }

  /* null parameter lists are valid so build the node here */
  if ((*pErr == aErrNone)
      && (*ppNewTree == NULL)) {
    *pErr = aASTNode_Create((aAST*)pSAST, tParameterList, ppNewTree);
    if (*pErr == aErrNone)
      bMatched = aTrue;
  }

  return bMatched;

} /* sSAST_ParseParameterList */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseParameterDeclaration
 */

aBool sSAST_ParseParameterDeclaration(aSAST* pSAST,
				     aASTNode** ppNewTree,
				     aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pDeclarationSpecifier = NULL;
  aASTNode* pDeclarator = NULL;
  aBool bDeclaratorRequired = aTrue;

  /* required declaration specifiers */
  if (*pErr == aErrNone)
    aAST_ParseDeclarationSpecifier((aAST*)pSAST, 
    				   &pDeclarationSpecifier, pErr);

  /* special case for void declaration specifiers */
  if ((*pErr == aErrNone)
      && (pDeclarationSpecifier)
      && (pDeclarationSpecifier->flags & fVoid))
    bDeclaratorRequired = aFalse;

  /* declarator */
  if ((*pErr == aErrNone) 
      && (pDeclarationSpecifier)
      && sSAST_ParseDeclarator(pSAST, &pDeclarator, 
      			       bDeclaratorRequired, pErr)) {

    bMatched = aTrue;
      
    /* we matched so build and populate the node */
    *pErr = aASTNode_Create((aAST*)pSAST, tParameterDeclaration, 
    			    ppNewTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pDeclarationSpecifier);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pDeclarator);
  }
  
  if ((bMatched == aFalse) 
      && (pDeclarationSpecifier != NULL)) {
    aASTNode_Destroy((aAST*)pSAST, pDeclarationSpecifier);
  }

  return bMatched;

} /* sSAST_ParseParameterDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseCompoundStatement
 */

aBool sSAST_ParseCompoundStatement(aSAST* pSAST, 
				  aASTNode** ppNewTree,
				  aErr* pErr)
{
  aBool bBracketFound = aFalse;
  aBool bMatched = aFalse;
  aASTNode* pDeclarationList = NULL;
  aASTNode* pStatementList = NULL;

  /* look for the open bracket */  
  if ((*pErr == aErrNone)
      && aAST_ParseSpecialChar((aAST*)pSAST, '{', aFalse, pErr)) {
    bBracketFound = aTrue;
  }

  /* now parse the declaration list (it is optional) */
  if ((*pErr == aErrNone)
      && (bBracketFound == aTrue)) {
     sSAST_ParseDeclarationList(pSAST, &pDeclarationList, pErr);
  }

  /* now parse the statement list (it is optional) */
  if ((*pErr == aErrNone)
      && (bBracketFound == aTrue)) {
     sSAST_ParseStatementList(pSAST, &pStatementList, pErr);
  }

  /* a hack for the end of file case */
  if ((*pErr == aErrEOF) && bBracketFound)
    *pErr = aErrNone;

  /* ensure the close bracket */
  if ((*pErr == aErrNone)
      && (bBracketFound == aTrue)
      && aAST_ParseSpecialChar((aAST*)pSAST, '}', aTrue, pErr)) {
    bMatched = aTrue;
  }

  /* if we matched, build the node */  
  if ((*pErr == aErrNone)
      && (bMatched == aTrue)) {
    *pErr = aASTNode_Create((aAST*)pSAST, tCompoundStatement, ppNewTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pDeclarationList);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pStatementList);
  } else {
    /* clean up if the parsing failed */
    if (pDeclarationList)
      aASTNode_Destroy((aAST*)pSAST, pDeclarationList);
    if (pStatementList)
      aASTNode_Destroy((aAST*)pSAST, pStatementList);
  }

  return bMatched;

} /* sSAST_ParseCompoundStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseDeclarationList
 */

aBool sSAST_ParseDeclarationList(aSAST* pSAST,
			        aASTNode** ppNewTree,
			        aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pDeclaration = NULL;

  /* gobble up each of the statements */
  while ((*pErr == aErrNone)
         && sSAST_ParseDeclaration(pSAST, &pDeclaration, pErr)) {

    /* since we got a valid declaration, create a list to work with */
    if ((*ppNewTree) == NULL)
      *pErr = aASTNode_Create((aAST*)pSAST, tDeclarationList, ppNewTree);

    /* now add the new statement to the list */
    if (*pErr == aErrNone) {
      *pErr = aASTNode_AddChild(*ppNewTree, pDeclaration);
      if (*pErr == aErrNone)
        bMatched = aTrue;
    }
  }

  return bMatched;

} /* sSAST_ParseDeclarationList */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseDeclaration
 */

aBool sSAST_ParseDeclaration(aSAST* pSAST,
			    aASTNode** ppNewTree,
			    aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pDeclSpec = NULL;
  aASTNode* pDeclaratorList = NULL;

  /* see if it is a valid statement */
  if ((*pErr == aErrNone)
      && aAST_ParseDeclarationSpecifier((aAST*)pSAST, &pDeclSpec, pErr)
      && sSAST_ParseInitDeclaratorList(pSAST, &pDeclaratorList, pErr)
      && aAST_ParseSpecialChar((aAST*)pSAST, ';', aTrue, pErr)) {
    bMatched = aTrue;
  }

  /* we matched so build and populate the node */
  if ((*pErr == aErrNone) 
      && (bMatched == aTrue)) {
    *pErr = aASTNode_Create((aAST*)pSAST, tDeclaration, ppNewTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pDeclSpec);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pDeclaratorList);
  } else {
    if (pDeclSpec)
      aASTNode_Destroy((aAST*)pSAST, pDeclSpec);
    if (pDeclaratorList)
      aASTNode_Destroy((aAST*)pSAST, pDeclaratorList);
  }

  return bMatched;

} /* sSAST_ParseDeclaration */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseStatementList
 */

aBool sSAST_ParseStatementList(aSAST* pSAST,
			      aASTNode** ppNewTree,
			      aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pStatement = NULL;

  /* gobble up each of the statements */
  while ((*pErr == aErrNone)
         && sSAST_ParseStatement(pSAST, &pStatement, pErr)) {

    /* since we got a valid statement, create a list to work with */
    if ((*ppNewTree) == NULL)
      *pErr = aASTNode_Create((aAST*)pSAST, tStatementList, ppNewTree);

    /* now add the new statement to the list */
    if (*pErr == aErrNone) {
      *pErr = aASTNode_AddChild(*ppNewTree, pStatement);
      if (*pErr == aErrNone)
        bMatched = aTrue;
    }
  }

  return bMatched;

} /* sSAST_ParseStatementList */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseStatement
 */

aBool sSAST_ParseStatement(aSAST* pSAST,
			  aASTNode** ppNewTree,
			  aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree = NULL;

  /* see if it is a valid statement */
  if ((*pErr == aErrNone)
      && (sSAST_ParseAsmStatement(pSAST, &pSubTree, pErr)
          || sSAST_ParseJumpStatement(pSAST, &pSubTree, pErr)
          || sSAST_ParseSelectionStatement(pSAST, &pSubTree, pErr)
          || sSAST_ParseLoopStatement(pSAST, &pSubTree, pErr)
          || sSAST_ParseLabeledStatement(pSAST, &pSubTree, pErr)
          || sSAST_ParseExpressionStatement(pSAST, &pSubTree, pErr)
          || sSAST_ParseCompoundStatement(pSAST, &pSubTree, pErr)
         )) {
    bMatched = aTrue;
  }

  /* we matched so build and populate the node */
  if ((*pErr == aErrNone) 
      && (bMatched == aTrue)) {
    *pErr = aASTNode_Create((aAST*)pSAST, tStatement, ppNewTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
  }

  return bMatched;

} /* sSAST_ParseStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseAsmStatement
 */

aBool sSAST_ParseAsmStatement(aSAST* pSAST,
			     aASTNode** ppNewTree,
			     aErr* pErr)
{
  aBool bMatched = aFalse;
  aBool bAsmFound = aFalse;
  aBool bBracketFound = aFalse;
  aASTNode* pAsmList = NULL;
  aToken* pToken;

  /* check for the asm starting token */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
      			  &pToken, NULL)) {

    /* we got a token so see if it is the asm start */
    if ((pToken->eType != tkIdentifier)
        || (aStringCompare(pToken->v.identifier, aTR_ASM) != 0)) {

      /* push it back if it is not the right token */
      aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
    } else {
      aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
      bAsmFound = aTrue;
    }
  }

  /* look for the open bracket */
  if ((*pErr == aErrNone)
      && (bAsmFound == aTrue)
      && aAST_ParseSpecialChar((aAST*)pSAST, '{', aTrue, pErr)) {
    bBracketFound = aTrue;
  }

  /* now parse asm list (it is optional) */
  if ((*pErr == aErrNone)
      && (bBracketFound == aTrue)) {
     sSAST_ParseAsmList(pSAST, &pAsmList, pErr);
  }

  /* ensure the close bracket */
  if ((*pErr == aErrNone)
      && (bAsmFound == aTrue)
      && aAST_ParseSpecialChar((aAST*)pSAST, '}', aTrue, pErr)) {
    bMatched = aTrue;
  }

  /* if we matched, build the node and install the symbol 
   * table 
   */
  if ((*pErr == aErrNone)
      && (bMatched == aTrue)) {
    *pErr = aASTNode_Create((aAST*)pSAST, tAsmStatement, ppNewTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pAsmList);
  }

  /* otherwise, clean up any assembly list that was created */
  if ((*pErr == aErrNone) && (bMatched == aFalse) && (pAsmList))
    aASTNode_Destroy((aAST*)pSAST, pAsmList);

  return bMatched;

} /* sSAST_ParseAsmStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseJumpStatement
 */

aBool sSAST_ParseJumpStatement(aSAST* pSAST,
			      aASTNode** ppNewTree,
			      aErr* pErr)
{
  aBool bMatched = aFalse;
  aToken* pToken;
  aSFlags flags = 0;
  aASTNode* pSubNode = NULL;

  /* check for the reserved tokens */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
      			  &pToken, NULL)) {
    if (pToken->eType == tkIdentifier) {
      if (aStringCompare(pToken->v.identifier, aTR_RETURN) == 0)
        flags |= (fJumpReturn | fPushVal);
      else if (aStringCompare(pToken->v.identifier, aTR_CONTINUE) == 0)
        flags |= fJumpContinue;
      else if (aStringCompare(pToken->v.identifier, aTR_BREAK) == 0)
        flags |= fJumpBreak;
      else if (aStringCompare(pToken->v.identifier, aTR_GOTO) == 0)
        flags |= fJumpGoto;
    }
    /* put the token back if we didn't get a jump reserved */
    if (flags == 0)
      aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
    else {
      *pErr = aASTNode_Create((aAST*)pSAST, tJumpStatement, ppNewTree);
      (*ppNewTree)->pToken = pToken;
    }
  }

  /* special case for return which has an optional expression */
  if ((*pErr == aErrNone)
      && (flags & fJumpReturn)
      && sSAST_ParseExpression(pSAST, &pSubNode, pErr)) {
  }

  /* special case for goto which has a required identifier */
  if ((*pErr == aErrNone)
      && (flags & fJumpGoto)) {
    if (sSAST_ParseIdentifier(pSAST, &pSubNode, pErr)) {
      pSubNode->flags |= fAddress;
    } else {
      /* if the label identifier is missing, report it and clean up */
      flags = 0;
      *pErr = aAST_OutputError((aAST*)pSAST, *ppNewTree, 
      			     aTE_MISSING_GOTO_ADDRESS);
      if (*pErr == aErrNone)
        aASTNode_Destroy((aAST*)pSAST, *ppNewTree);
      *ppNewTree = NULL;
    }
  }

  /* ensure the ending semicolon */
  if ((*pErr == aErrNone)
      && (flags != 0)
      && aAST_ParseSpecialChar((aAST*)pSAST, ';', aTrue, pErr)) {
    bMatched = aTrue;
  }

  /* if we matched, build the node and add in the sub node
   * when present
   */
  if (bMatched == aTrue) {
    if (*pErr == aErrNone) {
      aAssert(ppNewTree);
      (*ppNewTree)->flags |= flags;
      if (pSubNode != NULL)
        *pErr = aASTNode_AddChild(*ppNewTree, pSubNode);
    }
  }

  return bMatched;

} /* sSAST_ParseJumpStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseAsmList
 */

aBool sSAST_ParseAsmList(aSAST* pSAST,
			aASTNode** ppNewTree,
			aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;

  while ((*pErr == aErrNone)
         && (sSAST_ParseOpcode(pSAST, &pSubTree, pErr)
             || sSAST_ParseLabeledStatement(pSAST, &pSubTree, pErr))) {

    if (*ppNewTree == NULL)
      *pErr = aASTNode_Create((aAST*)pSAST, tAsmList, ppNewTree);

    if (*pErr == aErrNone) {
      *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
      if (*pErr == aErrNone)
        bMatched = aTrue;
    }
  }

  return bMatched;

} /* sSAST_ParseAsmList */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseOpcode
 */

aBool sSAST_ParseOpcode(aSAST* pSAST,
		       aASTNode** ppNewTree,
		       aErr* pErr)
{
  aBool bMatched = aFalse;
  aBool bParamsSupplied = aFalse;
  tOpCode opCode = op_INVALID;
  aASTNode* pExpression = NULL;
  aToken* pToken;
  int opLen = 0;

  
  /* check for an opcode */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
      			  &pToken, NULL)) {
    if (pToken->eType == tkIdentifier)
      opCode = aTEA_TextToOpCode(pToken->v.identifier);
    if (opCode == op_INVALID) {
      /* push it back if it is not the right token */
      aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
    }
  }

  if ((*pErr == aErrNone)
      && (opCode != op_INVALID)) {
    opLen = pSAST->opLengths[opCode];
    if (opLen == 0)
      bParamsSupplied = aTrue;
    else {
      if (aTEA_OpCodeTakesAddress(opCode) == aTrue) {
        /* get the identifier */
        if (sSAST_ParseIdentifier(pSAST, &pExpression, pErr)) {
          /* mark this identifier as an address */
          pExpression->flags |= fAddress;
          /* if it was a call, set the appropriate flags */
          if (opCode == op_CALL)
            pExpression->flags |= fIdentRtnName | fIdentAsmCall;
          bParamsSupplied = aTrue;
        }
      } else {
        /* get the expression */
        if (sSAST_ParseExpression(pSAST, &pExpression, pErr)) {
          bParamsSupplied = aTrue;
        }
      }
    }
  }

  /* build the node if we succeeded */
  if ((*pErr == aErrNone) 
      && (opCode != op_INVALID)) {
    *pErr = aASTNode_Create((aAST*)pSAST, tOpcode, ppNewTree);
    if (*pErr == aErrNone) {
      (*ppNewTree)->pToken = pToken;
      (*ppNewTree)->t.opcode.asmCode.code[0] = (char)opCode;
      (*ppNewTree)->t.opcode.asmCode.opLen = (char)(opLen + 1);
      if (pExpression != NULL) {
        *pErr = aASTNode_AddChild(*ppNewTree, pExpression);
      }
      if (*pErr == aErrNone)
        bMatched = aTrue;
    }
  }

  /* be sure the required opcode was provided */
  if ((bMatched == aTrue) 
	  && (bParamsSupplied == aFalse) 
	  && (opLen > 0)) {
    if (opLen == 1) {
      aAST_OutputError((aAST*)pSAST, *ppNewTree, 
    		     aTE_BYTE_PARAM_EXPECTED);
    } else {
      if (aTEA_OpCodeTakesAddress(opCode) == aTrue)
        aAST_OutputError((aAST*)pSAST, *ppNewTree, 
    		       aTE_ADDR_PARAM_EXPECTED);
      else
        aAST_OutputError((aAST*)pSAST, *ppNewTree, 
    		       aTE_SHORT_PARAM_EXPECTED);
    }
  }

  return bMatched;

} /* sSAST_ParseOpcode */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseExpression
 */

aBool sSAST_ParseExpression(aSAST* pSAST,
		      	   aASTNode** ppNewTree,
		       	   aErr* pErr)
{
  aBool bMatched = aFalse;
  aBool bExpressionDone = aFalse;
  aASTNode* pSubExpression = NULL;

  *ppNewTree = NULL;

  /* gobble up each of the parameters */
  while ((*pErr == aErrNone)
         && (bExpressionDone == aFalse)
         && sSAST_ParseAssignmentExpression(pSAST, &pSubExpression, pErr)) {

    /* since we got a valid expression, create a list to work with */
    if ((*pErr == aErrNone)
        && (*ppNewTree == NULL)) {
      *pErr = aASTNode_Create((aAST*)pSAST, tExpression, ppNewTree);
    }

    /* now add the new expression to the list */
    if (*pErr == aErrNone) {
      aAssert(*ppNewTree);
      *pErr = aASTNode_AddChild(*ppNewTree, pSubExpression);
      if (*pErr == aErrNone) {
        pSubExpression = NULL;
        bMatched = aTrue;
      }
    }
    
    /* now check for the comma separating the sub-expressions */
    if ((*pErr == aErrNone)
        && !aAST_ParseSpecialChar((aAST*)pSAST, ',', aFalse, pErr))
      bExpressionDone = aTrue;
  }

  return bMatched;

} /* sSAST_ParseExpression */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseString
 */

aBool sSAST_ParseString(aSAST* pSAST,
		       aASTNode** ppNewTree,
		       aErr* pErr)
{
  aBool bMatched = aFalse;
  aToken* pToken;

  /* check for a constant */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
      			  &pToken, NULL)) {
    if (pToken->eType == tkString)  {
      *pErr = aASTNode_Create((aAST*)pSAST, tString, ppNewTree);
      if (*pErr == aErrNone) {
        (*ppNewTree)->flags |= fConstant | fString;
        (*ppNewTree)->t.string.length = 
        		(unsigned char)aStringLen(pToken->v.string);
        (*ppNewTree)->pToken = pToken;
        bMatched = aTrue;
      }
    } else {
      aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, 
      			  pToken, pErr);
    }
  }

  return bMatched;

} /* sSAST_ParseString */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseIdentifier
 */

aBool sSAST_ParseIdentifier(aSAST* pSAST,
			   aASTNode** ppNewTree,
			   aErr* pErr)
{
  aBool bMatched = aFalse;
  aToken* pToken;

  /* check for an identifier */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
      			  &pToken, NULL)) {
    if (pToken->eType == tkIdentifier) {
      *pErr = aASTNode_Create((aAST*)pSAST, tIdentifier, ppNewTree);
      if (*pErr == aErrNone) {
        (*ppNewTree)->pToken = pToken;
        bMatched = aTrue;
      }
    } else {
      aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, 
      			  pToken, pErr);
    }
  }

  return bMatched;

} /* sSAST_ParseIdentifier */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseLabeledStatement
 */

aBool sSAST_ParseLabeledStatement(aSAST* pSAST,
				 aASTNode** ppNewTree,
				 aErr* pErr)
{
  aBool bMatched = aFalse;
  aToken* pToken = NULL;
  aToken* pColon = NULL;
  aASTNode* pExpression = NULL;
  aSFlags flags = 0;

  /* evaluate the first token */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
      			  &pToken, NULL)) {
    if (pToken->eType == tkIdentifier) {
      if (aStringCompare(pToken->v.identifier, aTR_CASE) == 0)
        flags |= fLabelCase;
      else if (aStringCompare(pToken->v.identifier, aTR_DEFAULT) == 0)
        flags |= fLabelDefault;
      else
        flags |= fLabelGeneral;
    }
    
    /* case needs an expression to follow (constant) */
    if ((flags & fLabelCase)
        && !sSAST_ParseExpression(pSAST, &pExpression, pErr)) {
      flags = 0;
      *pErr = aAST_OutputError(pSAST, NULL, aTE_BAD_CASE);
    }

    /* here if we have flags, we are ok so far */
    if ((*pErr == aErrNone)
        && flags 
        && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
        		    &pColon, NULL)) {
      if ((pColon->eType == tkSpecial)
          && (pColon->v.special == ':')) {
        bMatched = aTrue;
        aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pColon, pErr);
      } else {
        aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pColon, pErr);
      }
    }
  }

  /* build the node if we matched */
  if ((*pErr == aErrNone)
      && (bMatched == aTrue)) {
    *pErr = aASTNode_Create((aAST*)pSAST, tLabeledStatement, ppNewTree);
    if (*pErr == aErrNone) {
      (*ppNewTree)->flags |= flags;
      (*ppNewTree)->pToken = pToken;
      if (pExpression)
        *pErr = aASTNode_AddChild(*ppNewTree, pExpression);
    }
  }
  
  if (pToken && (bMatched != aTrue))
    aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);

  return bMatched;

} /* sSAST_ParseLabeledStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseExpressionStatement
 */

aBool sSAST_ParseExpressionStatement(aSAST* pSAST,
				    aASTNode** ppNewTree,
				    aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pExpression = NULL;

  /* expressions are optional */
  if (*pErr == aErrNone)
    sSAST_ParseExpression(pSAST, &pExpression, pErr);

  /* check for the semicolon */
  if ((*pErr == aErrNone)
      && aAST_ParseSpecialChar((aAST*)pSAST, ';', aFalse, pErr))
    bMatched = aTrue;

  /* build the node if we have an expression statement */
  if ((*pErr == aErrNone)
      && (bMatched == aTrue)) {
    *pErr = aASTNode_Create((aAST*)pSAST, tExpressionStatement, ppNewTree);
    if ((*pErr == aErrNone)
        && (pExpression != NULL))
      aASTNode_AddChild(*ppNewTree, pExpression);
  }
  
  if ((bMatched == aFalse) && pExpression)
    aASTNode_Unwind((aAST*)pSAST, pExpression);

  return bMatched;

} /* sSAST_ParseExpressionStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseAssignmentExpression
 */

aBool sSAST_ParseAssignmentExpression(aSAST* pSAST,
				     aASTNode** ppNewTree,
				     aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree = NULL;
  aASTNode* pAssignmentOperator = NULL;
  aASTNode* pAssignmentExpression = NULL;

  /* first check for a conditional-expression */
  if ((*pErr == aErrNone)
      && sSAST_ParseConditionalExpression(pSAST, &pSubTree, pErr)) {
    bMatched = aTrue;
    *ppNewTree = pSubTree;
  }

  /* then check for a standard assignment expression */
  if ((*pErr == aErrNone)
      && (bMatched == aTrue)
      && sSAST_ParseAssignmentOperator(pSAST, &pAssignmentOperator, pErr)
      && sSAST_ParseAssignmentExpression(pSAST, &pAssignmentExpression, pErr)) {
    bMatched = aTrue;
    *pErr = aASTNode_Create((aAST*)pSAST, tAssignmentExpression, ppNewTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pAssignmentOperator);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pAssignmentExpression);
  } else {
    /* clean up? */
  }
  
  return bMatched;

} /* sSAST_ParseAssignmentExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseUnaryExpression
 */

aBool sSAST_ParseUnaryExpression(aSAST* pSAST,
				aASTNode** ppNewTree,
				aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pPostFixExpression = NULL;
  aASTNode* pUnaryOperation = NULL;
  aASTNode* pCastExpression = NULL;
  aToken* pToken1 = NULL;
  aToken* pToken2 = NULL;
  aSFlags flags = 0;

  /* check for pre-increment and pre-decrement */
  if (!aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
                       &pToken1, NULL)) {
    if (pToken1->eType == tkSpecial) {
      switch (pToken1->v.special) {

      /* possibly a unary pre-increment */
      case '+':
        if (!aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
        		     &pToken2, NULL)
            && (pToken2->eType == tkSpecial)
            && (pToken2->v.special == '+'))
          flags = fUnaryPreInc;
        break;

      /* possibly a unary pre-decrement */
      case '-':
        if (!aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
        		     &pToken2, NULL)
            && (pToken2->eType == tkSpecial)
            && (pToken2->v.special == '-'))
          flags = fPostFixDec;
        break;
      } /* switch */
    }
    if (flags == 0) {
      if (pToken2)
        aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken2, pErr);
      if (pToken1) {
        aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken1, pErr);
        pToken1 = NULL;
      }
    } else {
      if (pToken2)
        aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken2, pErr);
    }
  }

  if (sSAST_ParsePostFixExpression(pSAST, &pPostFixExpression, pErr)) {
    bMatched = aTrue;
    if (flags) {
      *pErr = aASTNode_Create((aAST*)pSAST, tUnaryExpression, ppNewTree);
      if (*pErr == aErrNone) {
        (*ppNewTree)->pToken = pToken1;
        pToken1 = NULL;
        (*ppNewTree)->flags |= flags;
        *pErr = aASTNode_AddChild(*ppNewTree, pPostFixExpression);
      }
    } else {
      if (pToken1)
        aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken1, pErr);
      *ppNewTree = pPostFixExpression;
    }
  } else {
    if (flags && (*pErr == aErrNone))
      *pErr = aAST_OutputError(pSAST, NULL, aTE_EXPRESSION_EXPECTED);
  }

  if ((*pErr == aErrNone)
      && (bMatched == aFalse)
      && (sSAST_ParseUnaryOperator(pSAST, &pUnaryOperation, pErr)
          && sSAST_ParseCastExpression(pSAST, &pCastExpression, pErr))) {
    *pErr = aASTNode_Create((aAST*)pSAST, tUnaryExpression, ppNewTree);

    if (*pErr == aErrNone) {
      if (pPostFixExpression != NULL) {
        *pErr = aASTNode_AddChild(*ppNewTree, pPostFixExpression);
      }
      if (pUnaryOperation != NULL) {
        *pErr = aASTNode_AddChild(*ppNewTree, pUnaryOperation);
        if (*pErr == aErrNone)
          *pErr = aASTNode_AddChild(*ppNewTree, pCastExpression);
        if (*pErr == aErrNone)
          bMatched = aTrue;
      }
    }
  }

  return bMatched;

} /* sSAST_ParseUnaryExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseAssignmentOperator
 */

aBool sSAST_ParseAssignmentOperator(aSAST* pSAST,
				   aASTNode** ppNewTree,
				   aErr* pErr)
{
  aBool bMatched = aFalse;
  aToken* pToken;

  /* check for the asm starting token */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
      			  &pToken, NULL)) {
    aSFlags flags = 0;
    if (pToken->eType == tkSpecial) {
      switch (pToken->v.special) {
      case '=':
        flags |= fAssignOpEqual;
        break;
      } /* switch */
    }
    if (flags == 0)
      /* push it back if it is not the right token */
      aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
    else {
      aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
      if (*pErr == aErrNone)
        *pErr = aASTNode_Create((aAST*)pSAST, tAssignmentOperator, ppNewTree);
      if (*pErr == aErrNone) {
        (*ppNewTree)->flags |= flags;
        bMatched = aTrue;
      }
    }
  }

  return bMatched;

} /* sSAST_ParseAssignmentOperator */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParsePostFixExpression
 */

aBool sSAST_ParsePostFixExpression(aSAST* pSAST,
				  aASTNode** ppNewTree,
				  aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;
  aSFlags flags = 0;
  aASTNode* pArguments = NULL;

  if (sSAST_ParsePrimaryExpression(pSAST, &pSubTree, pErr)) {
    aToken* pToken1 = NULL;
    aToken* pToken2 = NULL;

    bMatched = aTrue;
    *ppNewTree = pSubTree;

    /* check for unary operators */
    if (!aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
    			 &pToken1, NULL)) {
      if (pToken1->eType == tkSpecial) {
        switch (pToken1->v.special) {

        /* possibly a unary post-increment */
        case '+':
          if (!aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
          		       &pToken2, NULL)) {
            if ((pToken2->eType == tkSpecial)
                && (pToken2->v.special == '+'))
              flags = fPostFixInc;
          }
          break;
        
        /* possibly a unary post-decrement */
        case '-':
          if (!aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
          		       &pToken2, NULL)) {
            if ((pToken2->eType == tkSpecial)
                && (pToken2->v.special == '-'))
              flags = fPostFixDec;
          }
          break;

        /* possibly a function call */
        case '(':
          if (sSAST_ParseArgumentExpressionList(pSAST, &pArguments, pErr)
              && aAST_ParseSpecialChar((aAST*)pSAST, ')', aTrue, pErr)) {
            flags = fPostFixRoutine;
          }
            break;

        } /* switch */
      }
      if (flags == 0) {
        if (pToken2)
          aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken2, pErr);
        if (pToken1)
          aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken1, pErr);
      } else {
        if (pToken1)
          aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken1, pErr);
        if (pToken2)
          aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken2, pErr);
      }
    }

    if (flags != 0) {
      *pErr = aASTNode_Create((aAST*)pSAST, tPostFixExpression, ppNewTree);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
      if (*pErr == aErrNone) {
        (*ppNewTree)->flags |= flags;
        if (pArguments)
          aASTNode_AddChild(*ppNewTree, pArguments);
      }
    }
  }

  return bMatched;

} /* sSAST_ParsePostFixExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseArgumentExpressionList
 */

aBool sSAST_ParseArgumentExpressionList(aSAST* pSAST,
				       aASTNode** ppNewTree,
				       aErr* pErr)
{
  aBool bMatched = aFalse;
  aBool bListDone = aFalse;
  aASTNode* pArgument = NULL;

  *ppNewTree = NULL;

  /* gobble up each of the parameters */
  while ((*pErr == aErrNone)
	 && (bListDone == aFalse)
         && sSAST_ParseAssignmentExpression(pSAST, &pArgument, pErr)) {

    /* since we got a valid parameter, create a list to work with */
    if ((*pErr == aErrNone)
        && (*ppNewTree == NULL)) {
      *pErr = aASTNode_Create((aAST*)pSAST, tArgumentExpressionList, ppNewTree);
    }

    /* now add the new parameter to the list */
    if (*pErr == aErrNone) {
      aAssert(*ppNewTree);
      *pErr = aASTNode_AddChild(*ppNewTree, pArgument);
      if (*pErr == aErrNone) {
        pArgument = NULL;
        bMatched = aTrue;
      }
    }

    /* now check for the comma separating the parameters */
    if ((*pErr == aErrNone)
        && !aAST_ParseSpecialChar((aAST*)pSAST, ',', aFalse, pErr)) {
      bListDone = aTrue;
    }
  }

  /* null argument lists are valid so build the node here */
  if ((*pErr == aErrNone)
      && (*ppNewTree == NULL)) {
    *pErr = aASTNode_Create((aAST*)pSAST, tArgumentExpressionList, ppNewTree);
    if (*pErr == aErrNone)
      bMatched = aTrue;
  }

  return bMatched;

} /* sSAST_ParseArgumentExpressionList */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParsePrimaryExpression
 */

aBool sSAST_ParsePrimaryExpression(aSAST* pSAST,
				  aASTNode** ppNewTree,
				  aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;

  if ((*pErr == aErrNone)
      && (sSAST_ParseIdentifier(pSAST, &pSubTree, pErr)
          || aAST_ParseConstant((aAST*)pSAST, &pSubTree, pErr)
          || sSAST_ParseString(pSAST, &pSubTree, pErr)
          || (aAST_ParseSpecialChar((aAST*)pSAST, '(', aFalse, pErr)
              && sSAST_ParseExpression(pSAST, &pSubTree, pErr)
              && aAST_ParseSpecialChar((aAST*)pSAST, ')', aTrue, pErr)))) {
    *pErr = aASTNode_Create((aAST*)pSAST, tPrimaryExpression, ppNewTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
    if (*pErr == aErrNone)
      bMatched = aTrue;
  }

  return bMatched;

} /* sSAST_ParsePrimaryExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseSelectionStatement
 */

aBool sSAST_ParseSelectionStatement(aSAST* pSAST,
				   aASTNode** ppNewTree,
				   aErr* pErr)
{
  aBool bMatched = aFalse;
  aToken* pToken;
  aASTNode* pExpression = NULL;
  aASTNode* pStatement = NULL;
  aSFlags flags = 0;
  
  /* initialize */
  aAssert(ppNewTree);
  *ppNewTree = NULL;

  /* check for the reserved start first */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, &pToken, NULL)) {
    if (pToken->eType == tkIdentifier) {
      if (aStringCompare(pToken->v.identifier, aTR_IF) == 0)
        flags |= fSelectIf;
      else if (aStringCompare(pToken->v.identifier, aTR_SWITCH) == 0)
        flags |= fSelectSwitch;
    }

    /* put the token back if we didn't get a selection reserved */
    if (flags == 0)
      aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
    else {
      /* store the token for use in error reporting, etc */
      *pErr = aASTNode_Create((aAST*)pSAST, tSelectionStatement, ppNewTree);
      if (*pErr == aErrNone) {
        (*ppNewTree)->pToken = pToken;
        (*ppNewTree)->flags |= flags;
      }
    }
  }
  
  /* everybody has an expression */
  if ((*pErr == aErrNone)
      && (flags != 0)
      && aAST_ParseSpecialChar((aAST*)pSAST, '(', aTrue, pErr)
      && sSAST_ParseExpression(pSAST, &pExpression, pErr)
      && aAST_ParseSpecialChar((aAST*)pSAST, ')', aTrue, pErr)
      && sSAST_ParseStatement(pSAST, &pStatement, pErr))
    bMatched = aTrue;

  /* clean up the tokens either way */
  if ((bMatched == aTrue)
      && (*pErr == aErrNone)) {
    *pErr = aASTNode_AddChild(*ppNewTree, pExpression);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pStatement);
  } else {
    if (*ppNewTree != NULL) {
      aASTNode_Destroy((aAST*)pSAST, *ppNewTree);
      *ppNewTree = NULL;
    }
    if (pExpression != NULL)
      aASTNode_Destroy((aAST*)pSAST, pExpression);
    if (pStatement != NULL)
      aASTNode_Destroy((aAST*)pSAST, pStatement);
  }

  /* check for an optional else clause */  
  if ((bMatched == aTrue)
      && (*pErr == aErrNone)
      && (flags & fSelectIf)
      && aAST_ParseIdentifierToken((aAST*)pSAST, aTR_ELSE, aFalse, pErr)
      && sSAST_ParseStatement(pSAST, &pStatement, pErr)) {
    *pErr = aASTNode_AddChild(*ppNewTree, pStatement);
  }

  return bMatched;

} /* sSAST_ParseSelectionStatement */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseLoopStatement
 */

aBool sSAST_ParseLoopStatement(aSAST* pSAST,
			      aASTNode** ppNewTree,
			      aErr* pErr)
{
  aBool bMatched = aFalse;
  aToken* pToken;
  aASTNode* pPreExpr = NULL;
  aASTNode* pLoopExpr = NULL;
  aASTNode* pPostExpr = NULL;
  aASTNode* pStatement = NULL;
  aSFlags flags = 0;

  /* initialize */
  aAssert(ppNewTree);
  *ppNewTree = NULL;

  /* check for the loop start first */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, &pToken, NULL)) {
    if (pToken->eType == tkIdentifier) {
      if (aStringCompare(pToken->v.identifier, aTR_WHILE) == 0)
        flags |= fLoopWhile;
      else if (aStringCompare(pToken->v.identifier, aTR_DO) == 0)
        flags |= fLoopDoWhile;
      else if (aStringCompare(pToken->v.identifier, aTR_FOR) == 0)
        flags |= fLoopFor;
    }

    /* put the token back if we didn't get a loop reserved word */
    if (flags == 0)
      aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
    else {
      /* store the token for use in error reporting, etc */
      *pErr = aASTNode_Create((aAST*)pSAST, tLoopStatement, ppNewTree);
      if (*pErr == aErrNone) {
        (*ppNewTree)->pToken = pToken;
        (*ppNewTree)->flags |= flags;
      }
    }
  }

  /* everybody handles it differently */
  if (*pErr == aErrNone) {
    switch (flags) {

    case fLoopWhile:
      if (aAST_ParseSpecialChar((aAST*)pSAST, '(', aTrue, pErr)
          && sSAST_ParseExpression(pSAST, &pLoopExpr, pErr)
          && aAST_ParseSpecialChar((aAST*)pSAST, ')', aTrue, pErr)
          && sSAST_ParseStatement(pSAST, &pStatement, pErr))
      bMatched = aTrue;
      break;

    case fLoopDoWhile:
      if (sSAST_ParseStatement(pSAST, &pStatement, pErr)
          && aAST_ParseIdentifierToken((aAST*)pSAST, aTR_WHILE, aTrue, pErr)
      	  && aAST_ParseSpecialChar((aAST*)pSAST, '(', aTrue, pErr)
          && sSAST_ParseExpression(pSAST, &pLoopExpr, pErr)
          && aAST_ParseSpecialChar((aAST*)pSAST, ')', aTrue, pErr)
          && aAST_ParseSpecialChar((aAST*)pSAST, ';', aTrue, pErr))
      bMatched = aTrue;
      break;

    case fLoopFor:
      if (aAST_ParseSpecialChar((aAST*)pSAST, '(', aTrue, pErr)) {
        sSAST_ParseExpression(pSAST, &pPreExpr, pErr);
        if (aAST_ParseSpecialChar((aAST*)pSAST, ';', aTrue, pErr)) {
          sSAST_ParseExpression(pSAST, &pLoopExpr, pErr);
          if (aAST_ParseSpecialChar((aAST*)pSAST, ';', aTrue, pErr)) {
            sSAST_ParseExpression(pSAST, &pPostExpr, pErr);
            if (aAST_ParseSpecialChar((aAST*)pSAST, ')', aTrue, pErr)
                && sSAST_ParseStatement(pSAST, &pStatement, pErr))
              bMatched = aTrue;
          }
        }
      }
      break;
    } /* switch */
  }

  /* clean up the tokens either way */
  if ((bMatched == aTrue)
      && (*pErr == aErrNone)) {
    if (pPreExpr) {
      pPreExpr->flags |= fLoopPre;
      *pErr = aASTNode_AddChild(*ppNewTree, pPreExpr);
    }
    if (pLoopExpr) {
      pLoopExpr->flags |= fLoopCondition;
      *pErr = aASTNode_AddChild(*ppNewTree, pLoopExpr);
    }
    if (pPostExpr) {
      pPostExpr->flags |= fLoopPost;
      *pErr = aASTNode_AddChild(*ppNewTree, pPostExpr);
    }
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pStatement);
  } else {
    if (*ppNewTree != NULL) {
      aASTNode_Destroy((aAST*)pSAST, *ppNewTree);
      *ppNewTree = NULL;
    }
    if (pPreExpr != NULL)
      aASTNode_Destroy((aAST*)pSAST, pPreExpr);
    if (pLoopExpr != NULL)
      aASTNode_Destroy((aAST*)pSAST, pLoopExpr);
    if (pPostExpr != NULL)
      aASTNode_Destroy((aAST*)pSAST, pPostExpr);
    if (pStatement != NULL)
      aASTNode_Destroy((aAST*)pSAST, pStatement);
  }

  return bMatched;

} /* sSAST_ParseLoopStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseInitializer
 */

aBool sSAST_ParseInitializer(aSAST* pSAST,
			    aASTNode** ppNewTree,
			    aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;

  if ((*pErr == aErrNone)
      && sSAST_ParseAssignmentExpression(pSAST, &pSubTree, pErr)) {
    *pErr = aASTNode_Create((aAST*)pSAST, tInitializer, ppNewTree);
    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
    if (*pErr == aErrNone)
      bMatched = aTrue;
  }

  return bMatched;

} /* sSAST_ParseInitializer */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseUnaryOperator
 */

aBool sSAST_ParseUnaryOperator(aSAST* pSAST,
			      aASTNode** ppNewTree,
			      aErr* pErr)
{
  aBool bMatched = aFalse;
  aToken* pToken;

  /* check for the asm starting token */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, &pToken, NULL)) {
    aSFlags flags = 0;
    if (pToken->eType == tkSpecial) {
      switch (pToken->v.special) {
      case '-':
        flags |= fUnaryOpMinus;
        break;
      case '+':
        flags |= fUnaryOpPlus;
        break;
      case '!':
        flags |= fUnaryOpBang;
        break;
      case '~':
        flags |= fUnaryOpTilde;
        break;
      } /* switch */
    }
    if (flags == 0)
      /* push it back if it is not the right token */
      aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
    else {
      if (*pErr == aErrNone)
        *pErr = aASTNode_Create((aAST*)pSAST, tUnaryOperator, ppNewTree);
      if (*pErr == aErrNone) {
        (*ppNewTree)->pToken = pToken;
        (*ppNewTree)->flags |= flags;
        bMatched = aTrue;
      }
    }
  }

  return bMatched;

} /* sSAST_ParseUnaryOperator */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseConditionalExpression
 */

aBool sSAST_ParseConditionalExpression(aSAST* pSAST,
				      aASTNode** ppNewTree,
				      aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;
  aASTNode* pExpression = NULL;
  aASTNode* pConditionalExpression = NULL;

  if (sSAST_ParseLogicalORExpression(pSAST, &pSubTree, pErr)) {
    bMatched = aTrue;
    if (aAST_ParseSpecialChar((aAST*)pSAST, '?', aFalse, pErr)
        && sSAST_ParseExpression(pSAST, &pExpression, pErr)
        && aAST_ParseSpecialChar((aAST*)pSAST, ':', aTrue, pErr)
        && sSAST_ParseExpression(pSAST, &pConditionalExpression, pErr)) {
      *pErr = aASTNode_Create((aAST*)pSAST, tConditionalExpression, ppNewTree);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
      if ((*pErr == aErrNone)
          && (pExpression))
        *pErr = aASTNode_AddChild(*ppNewTree, pExpression);
      if ((*pErr == aErrNone)
          && (pConditionalExpression))
        *pErr = aASTNode_AddChild(*ppNewTree, pConditionalExpression);
    } else
      *ppNewTree = pSubTree;
  }

  return bMatched;

} /* sSAST_ParseConditionalExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseLogicalORExpression
 */

aBool sSAST_ParseLogicalORExpression(aSAST* pSAST,
				    aASTNode** ppNewTree,
				    aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;
  aASTNode* pSecond = NULL;

  if (sSAST_ParseLogicalANDExpression(pSAST, &pSubTree, pErr)) {
    bMatched = aTrue;
    if (aAST_ParseSpecialChar((aAST*)pSAST, '|', aFalse, pErr)
        && aAST_ParseSpecialChar((aAST*)pSAST, '|', aFalse, pErr)
        && sSAST_ParseLogicalORExpression(pSAST, &pSecond, pErr)) {
      *pErr = aASTNode_Create((aAST*)pSAST, tLogicalORExpression, ppNewTree);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
      if ((*pErr == aErrNone) && pSecond)
        *pErr = aASTNode_AddChild(*ppNewTree, pSecond);
    } else
      *ppNewTree = pSubTree;
  }

  return bMatched;

} /* sSAST_ParseLogicalORExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseLogicalANDExpression
 */

aBool sSAST_ParseLogicalANDExpression(aSAST* pSAST,
				     aASTNode** ppNewTree,
				     aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;
  aASTNode* pSecond = NULL;

  if (aAST_ParseInclusiveORExpression((aAST*)pSAST, &pSubTree, pErr)) {
    bMatched = aTrue;
    if (aAST_ParseSpecialChar((aAST*)pSAST, '&', aFalse, pErr)
        && aAST_ParseSpecialChar((aAST*)pSAST, '&', aFalse, pErr)
        && sSAST_ParseLogicalANDExpression(pSAST, &pSecond, pErr)) {
      *pErr = aASTNode_Create((aAST*)pSAST, tLogicalANDExpression, ppNewTree);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
      if ((*pErr == aErrNone) && pSecond)
        *pErr = aASTNode_AddChild(*ppNewTree, pSecond);
    } else
      *ppNewTree = pSubTree;
  }

  return bMatched;

} /* sSAST_ParseLogicalANDExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseEqualityExpression
 */

aBool sSAST_ParseEqualityExpression(aSAST* pSAST,
				   aASTNode** ppNewTree,
				   aErr* pErr)
{
  aBool bMatched = aFalse;
  aSFlags flags = 0;
  aASTNode* pSubTree;
  aASTNode* pSecond = NULL;
  aToken* pToken1 = NULL;
  aToken* pToken2 = NULL;
  aBool bTokensUsed = aFalse;

  if (sSAST_ParseRelationalExpression(pSAST, &pSubTree, pErr)) {
    bMatched = aTrue;

    /* look for the first equality character */
    if ((*pErr == aErrNone)
        && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
        		    &pToken1, NULL)) {
      if (pToken1->eType == tkSpecial) {
        switch (pToken1->v.special) {
        case '=': flags |= fEquality;		break;
        case '!': flags |= fNotEquality;	break;
        } /* switch */
      }
    }
    
    /* flags is non-zero if first character passed */

    /* look for the second equality character (=) */
    if ((*pErr == aErrNone)
        && (flags != 0)
        && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, 
        		    &pToken2, NULL)) {
      if ((pToken2->eType != tkSpecial)
          || (pToken2->v.special != '=')) {
        flags = 0;
      }
    }
    
    /* flags is zero if second character didn't pass */

    /* if we are ok, try to find the right side */
    if ((*pErr == aErrNone)
        && (flags != 0)) {
      if (sSAST_ParseEqualityExpression(pSAST, &pSecond, pErr)) {
        *pErr = aASTNode_Create((aAST*)pSAST, tEqualityExpression, 
      				ppNewTree);
        if (*pErr == aErrNone) {
          *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
          (*ppNewTree)->flags |= flags;
        }
        if ((*pErr == aErrNone) && pSecond)
          *pErr = aASTNode_AddChild(*ppNewTree, pSecond);
        bTokensUsed = aTrue;
      }
    }
  }

  if (*pErr == aErrNone) {
    if (bTokensUsed != aTrue) {
      *ppNewTree = pSubTree;
      if ((*pErr == aErrNone) && pToken2)
        aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken2, pErr);
      if (pToken1)
        aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken1, pErr);
    } else {
      if ((*pErr == aErrNone) && pToken2)
        aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken2, pErr);
      if (pToken1)
        aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken1, pErr);
    }
  }

  return bMatched;

} /* sSAST_ParseEqualityExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseRelationalExpression
 */

aBool sSAST_ParseRelationalExpression(aSAST* pSAST,
				     aASTNode** ppNewTree,
				     aErr* pErr)
{
  aBool bMatched = aFalse;
  aSFlags flags = 0;
  aASTNode* pSubTree;
  aASTNode* pSecond = NULL;
  aToken* pToken1 = NULL;
  aToken* pToken2 = NULL;
  aBool bTokensUsed = aFalse;

  if (sSAST_ParseShiftExpression(pSAST, &pSubTree, pErr)) {
    bMatched = aTrue;

    /* look for the first equality character */
    if ((*pErr == aErrNone)
        && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, &pToken1, NULL)) {
      if (pToken1->eType == tkSpecial) {
        switch (pToken1->v.special) {
        case '<': flags |= fRelateLT;		break;
        case '>': flags |= fRelateGT;		break;
        } /* switch */
      }
    }
    
    /* flags is non-zero if first character passed */

    /* look for the second equality character (=) */
    if ((*pErr == aErrNone)
        && (flags != 0)
        && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, &pToken2, NULL)) {
      if ((pToken2->eType == tkSpecial)
          && (pToken2->v.special == '=')) {
        flags |= fRelateEQ;
        aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken2, pErr);
      } else {
        aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken2, pErr);
      }
    }

    /* flags is zero operation didn't pass */

    /* if we are ok, try to find the right side */
    if ((*pErr == aErrNone)
        && (flags != 0)) {
      if (sSAST_ParseRelationalExpression(pSAST, &pSecond, pErr)) {
        *pErr = aASTNode_Create((aAST*)pSAST, tRelationalExpression, 
      				ppNewTree);
        if (*pErr == aErrNone) {
          *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
          (*ppNewTree)->flags |= flags;
          (*ppNewTree)->pToken = pToken1;
          pToken1 = NULL;
        }
        if ((*pErr == aErrNone) && pSecond)
          *pErr = aASTNode_AddChild(*ppNewTree, pSecond);
        bTokensUsed = aTrue;
      }
    }
  }

  if (*pErr == aErrNone) {
    if (bTokensUsed != aTrue) {
      *ppNewTree = pSubTree;
      if ((*pErr == aErrNone) && pToken1)
        aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken1, pErr);
    } else {
      if ((*pErr == aErrNone) && pToken1)
        aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken1, pErr);
    }
  }

  return bMatched;

} /* sSAST_ParseRelationalExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseShiftExpression
 */

aBool sSAST_ParseShiftExpression(aSAST* pSAST,
				aASTNode** ppNewTree,
				aErr* pErr)
{
  aBool bMatched = aFalse;
  aSFlags flags = 0;
  aASTNode* pSubTree;
  aASTNode* pSecond = NULL;
  aToken* pToken1 = NULL;
  aToken* pToken2 = NULL;
  aBool bTokensUsed = aFalse;

  if (sSAST_ParseAdditiveExpression(pSAST, &pSubTree, pErr)) {
    bMatched = aTrue;

    /* look for the first shift character */
    if ((*pErr == aErrNone)
        && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, &pToken1, NULL)) {
      if (pToken1->eType == tkSpecial) {
        switch (pToken1->v.special) {
        case '<': flags |= fShiftLeft;		break;
        case '>': flags |= fShiftRight;		break;
        } /* switch */
      }
    }
    
    /* flags is non-zero if first character passed */

    /* look for the second equality character (=) */
    if ((*pErr == aErrNone)
        && (flags != 0)
        && !aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, &pToken2, NULL)) {
      if (pToken2->eType == tkSpecial) {
        if (flags & fShiftLeft) {
          if (pToken2->v.special != '<')
            flags = 0;
        } else {
          if (pToken2->v.special != '>')
            flags = 0;
        }
      } else {
        flags = 0;
      }
      if (flags)
        aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken2, pErr);
      else
        aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken2, pErr);
    }

    /* flags is zero operation didn't pass */

    /* if we are ok, try to find the right side */
    if ((*pErr == aErrNone)
        && (flags != 0)) {
      if (sSAST_ParseShiftExpression(pSAST, &pSecond, pErr)) {
        *pErr = aASTNode_Create((aAST*)pSAST, tShiftExpression, ppNewTree);
        if (*pErr == aErrNone) {
          *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
          (*ppNewTree)->flags |= flags;
        }
        if ((*pErr == aErrNone) && pSecond)
          *pErr = aASTNode_AddChild(*ppNewTree, pSecond);
        bTokensUsed = aTrue;
      }
    }
  }

  if (*pErr == aErrNone) {
    if (bTokensUsed != aTrue) {
      *ppNewTree = pSubTree;
      if ((*pErr == aErrNone) && pToken1)
        aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken1, pErr);
    } else {
      if ((*pErr == aErrNone) && pToken1)
        aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken1, pErr);
    }
  }

  return bMatched;

} /* sSAST_ParseShiftExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_AdditiveOperator
 */

aSFlags sSAST_AdditiveOperator(aToken* pToken)
{
  aSFlags flags = 0;
  
  if (pToken->eType == tkSpecial) {
    switch (pToken->v.special) {
      case '+':	flags = f2OpPLUS;	break;
      case '-':	flags = f2OpMINUS;	break;
    } /* switch */
  }
  
  return flags;

} /* sSAST_AdditiveOperator */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseAdditiveExpression
 */

aBool sSAST_ParseAdditiveExpression(aSAST* pSAST,
				   aASTNode** ppNewTree,
				   aErr* pErr)
{
  return sL2REval(pSAST, 
  		  sSAST_ParseMultiplicativeExpression,
  		  sSAST_AdditiveOperator,
		  ppNewTree,
		  pErr);

} /* sSAST_ParseAdditiveExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_MultiplicativeOperator
 */

aSFlags sSAST_MultiplicativeOperator(aToken* pToken)
{
  aSFlags flags = 0;
  
  if (pToken->eType == tkSpecial) {
    switch (pToken->v.special) {
      case '*':	flags = f2OpMULT;	break;
      case '/':	flags = f2OpDIV;	break;
      case '%':	flags = f2OpMOD;	break;
    } /* switch */
  }
  
  return flags;

} /* sSAST_MultiplicativeOperator */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseMultiplicativeExpression
 */

aBool sSAST_ParseMultiplicativeExpression(aSAST* pSAST,
				         aASTNode** ppNewTree,
				         aErr* pErr)
{
  return sL2REval(pSAST, 
  		  sSAST_ParseCastExpression,
  		  sSAST_MultiplicativeOperator,
		  ppNewTree,
		  pErr);

} /* sSAST_ParseMultiplicativeExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSAST_ParseCastExpression
 */

aBool sSAST_ParseCastExpression(aSAST* pSAST,
			       aASTNode** ppNewTree,
			       aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;
  aToken* pToken;
  
  *ppNewTree = NULL;

  /* see if it is a cast expression */
  if (!aTokenizer_Next(pSAST->ioRef, pSAST->tokenizer, &pToken, NULL)) {
    if ((pToken->eType == tkSpecial)
        && (pToken->v.special == '(')
        && aAST_ParseDeclarationSpecifier((aAST*)pSAST, &pSubTree, pErr)
        && aAST_ParseSpecialChar((aAST*)pSAST, ')', aTrue, pErr)) {
      aTokenizer_Dispose(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
      *pErr = aASTNode_Create((aAST*)pSAST, tCastExpression, ppNewTree);
      if (*pErr == aErrNone)
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
    } else {
      aTokenizer_PushBack(pSAST->ioRef, pSAST->tokenizer, pToken, pErr);
    }
  }

  if (*pErr == aErrNone) {
    if (sSAST_ParseUnaryExpression(pSAST, &pSubTree, pErr)) {
      if (*ppNewTree == NULL)
        *ppNewTree = pSubTree;
      else
        *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);        
      bMatched = aTrue;
    }
  }

  return bMatched;

} /* sSAST_ParseCastExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sL2REval
 *
 * generic left-to-right production
 *
 * since all left-to-right production follows the same format, we 
 * just pass in the subexpression producer and operator parsing
 * and handle the evaluation consistently.
 */

aBool sL2REval(aSAST* pSAST,
	       pL2RProducerProc subexpression,
	       pL2RCombinerProc combiner,
	       aASTNode** ppNewTree,
	       aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pLeft;
  *ppNewTree = NULL;

  /* first, see if there is a left-hand side */
  if (subexpression(pSAST, &pLeft, pErr)) {
    aASTNode* pResult;
    do {
      aToken* pToken;
      aSFlags flags = 0;

      pResult = NULL;
      bMatched = aTrue;
      *ppNewTree = pLeft;

      /* now, see if there is a expression combiner */
      if (!aTokenizer_Next(pSAST->ioRef, 
      			   pSAST->tokenizer, 
      			   &pToken, 
      			   NULL)) {
        aASTNode* pRight;
        
        flags = combiner(pToken);

        if ((flags != 0)
            && subexpression(pSAST, 
            		     &pRight, 
            		     pErr)) {
          *pErr = aASTNode_Create((aAST*)pSAST, 
          			  tAdditiveExpression, 
      			          &pResult);
          if (*pErr == aErrNone) {
            pResult->flags |= flags;
            *pErr = aASTNode_AddChild(pResult, 
            			      pLeft);
          }
          if (*pErr == aErrNone)
            *pErr = aASTNode_AddChild(pResult, 
            			      pRight);
          aTokenizer_Dispose(pSAST->ioRef, 
          		     pSAST->tokenizer, 
          		     pToken, 
          		     pErr);
          pLeft = pResult;
        } else {
          aTokenizer_PushBack(pSAST->ioRef, 
          		      pSAST->tokenizer, 
          		      pToken, 
          		      pErr);
        }
      }
    } while (pResult != NULL);
  } /* if */

  return bMatched;

} /* sL2REval */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSAST_Create
 */

aErr aSAST_Create(aIOLib ioRef,
		  aSAST** ppSAST,
		  unsigned char* pOpLengths,
		  aTokenErrProc errProc,
		  void* errProcRef)
{
  aErr astErr = aErrNone;
  aSAST* pSAST;

  aAssert(ppSAST);
  *ppSAST = NULL;

  pSAST = (aSAST*)aMemAlloc(sizeof(aSAST));
  if (pSAST == NULL)
    astErr = aErrMemory;

  if (astErr == aErrNone) {
    aBZero(pSAST, sizeof(aSAST));
    pSAST->errProc = errProc;
    pSAST->errProcRef = errProcRef;
    pSAST->outputErrProc = aAST_OutputError;
    pSAST->ioRef = ioRef;
  }

  if (astErr == aErrNone)
    aMemPool_Create(ioRef, sizeof(aASTNode),
    		    aASTNODEPOOLSIZE, &pSAST->ASTNodePool,
    		    &astErr);

  if (astErr == aErrNone) {
    pSAST->opLengths = pOpLengths;
    pSAST->createNode = (ASTNodeCreateProc)aASTNode_Create;
    pSAST->addNodeChild = aASTNode_AddChild;
    *ppSAST = pSAST;
  }

  return astErr;

} /* aSAST_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSAST_Parse
 */

aErr aSAST_Parse(aSAST* pSAST,
		aTokenizerRef tokenizerRef,
		aStreamRef errStream)
{
  aErr astErr = aErrNone;
  
  aAssert(pSAST);
  aAssert(pSAST->ASTNodePool);

  /* set up the SAST */
  pSAST->tokenizer = tokenizerRef;
  pSAST->errStream = errStream;

  /* do the parse */
  if (astErr == aErrNone)
    sSAST_ParseTranslationUnit(pSAST, &pSAST->pTree, &astErr);

  return astErr;

} /* aSAST_Parse */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSAST_Destroy
 */

aErr aSAST_Destroy(aSAST* pSAST)
{
  aErr astErr = aErrNone;

  aAssert(pSAST);

  if (pSAST->pTree)
    aASTNode_Destroy((aAST*)pSAST, pSAST->pTree);

  if (pSAST->ASTNodePool != NULL) {
    aMemPool_Destroy(pSAST->ioRef, pSAST->ASTNodePool, &astErr);
    pSAST->ASTNodePool = NULL;
  }

  aMemFree((aMemPtr)pSAST);

  return astErr;

} /* aSAST_Destroy */




















/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_ParseInclusiveORExpression
 */

aBool aAST_ParseInclusiveORExpression(aAST* pAST,
				      aASTNode** ppNewTree,
				      aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;
  aASTNode* pSecond = NULL;
  aToken* pToken;

  if (aAST_ParseExclusiveORExpression(pAST, &pSubTree, pErr)) {
    bMatched = aTrue;
    *ppNewTree = pSubTree;
    if (!aTokenizer_Next(pAST->ioRef, pAST->tokenizer, &pToken, NULL)) {
      if ((pToken->eType == tkSpecial)
          && (pToken->v.special == '|')
          && aAST_ParseInclusiveORExpression(pAST, &pSecond, pErr)) {
        *pErr = aASTNode_Create((aAST*)pAST, tInclusiveORExpression, 
      			        ppNewTree);
        if (*pErr == aErrNone) {
          (*ppNewTree)->flags |= f2OpOR;
          *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
        }
        if ((*pErr == aErrNone) && (pSecond))
          *pErr = aASTNode_AddChild(*ppNewTree, pSecond);
        aTokenizer_Dispose(pAST->ioRef, pAST->tokenizer, pToken, pErr);
      } else {
        aTokenizer_PushBack(pAST->ioRef, pAST->tokenizer, pToken, pErr);
      }
    }
  }

  return bMatched;

} /* aAST_ParseInclusiveORExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_ParseExclusiveORExpression
 */

aBool aAST_ParseExclusiveORExpression(aAST* pAST,
				      aASTNode** ppNewTree,
				      aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;
  aASTNode* pSecond = NULL;
  aToken* pToken;

  if (aAST_ParseANDExpression(pAST, &pSubTree, pErr)) {
    bMatched = aTrue;
    *ppNewTree = pSubTree;
    if (!aTokenizer_Next(pAST->ioRef, pAST->tokenizer, &pToken, NULL)) {
      if ((pToken->eType == tkSpecial)
          && (pToken->v.special == '^')
          && aAST_ParseExclusiveORExpression(pAST, &pSecond, pErr)) {
        *pErr = aASTNode_Create((aAST*)pAST, tExclusiveORExpression, 
      			        ppNewTree);
        if (*pErr == aErrNone) {
          (*ppNewTree)->flags |= f2OpXOR;
          *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
        }
        if ((*pErr == aErrNone) && (pSecond))
          *pErr = aASTNode_AddChild(*ppNewTree, pSecond);
        aTokenizer_Dispose(pAST->ioRef, pAST->tokenizer, pToken, pErr);
      } else {
        aTokenizer_PushBack(pAST->ioRef, pAST->tokenizer, pToken, pErr);
      }
    }
  }

  return bMatched;

} /* aAST_ParseExclusiveORExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_ParseANDExpression
 */

aBool aAST_ParseANDExpression(aAST* pAST,
			      aASTNode** ppNewTree,
			      aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pSubTree;
  aASTNode* pSecond = NULL;
  aToken* pToken;

  if (sSAST_ParseEqualityExpression((aSAST*)pAST, &pSubTree, pErr)) {
    bMatched = aTrue;
    *ppNewTree = pSubTree;
    if (!aTokenizer_Next(pAST->ioRef, pAST->tokenizer, &pToken, NULL)) {
      if ((pToken->eType == tkSpecial)
          && (pToken->v.special == '&')
          && aAST_ParseANDExpression(pAST, &pSecond, pErr)) {
        *pErr = aASTNode_Create((aAST*)pAST, tANDExpression, 
      			        ppNewTree);
        if (*pErr == aErrNone) {
          (*ppNewTree)->flags |= f2OpAND;
          *pErr = aASTNode_AddChild(*ppNewTree, pSubTree);
        }
        if ((*pErr == aErrNone) && (pSecond))
          *pErr = aASTNode_AddChild(*ppNewTree, pSecond);
        aTokenizer_Dispose(pAST->ioRef, pAST->tokenizer, pToken, pErr);
      } else {
        aTokenizer_PushBack(pAST->ioRef, pAST->tokenizer, pToken, pErr);
      }
    }
  }

  return bMatched;

} /* aAST_ParseANDExpression */
