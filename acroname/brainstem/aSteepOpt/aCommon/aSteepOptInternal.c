/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSteepOptInternal.c					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of backend code generator for the   */
/*		the TEA virtual machine. 	                   */
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
#include "aAST.h"
#include "aSteepText.h"
#include "aSteepOptInternal.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aBool sTypeCanPromote(aSFlags lValue, 
			     aSFlags rValue);
static aErr sBuildAutoCast(aSAST* pSAST,
			   aSFlags type, 
			   aASTNode** ppNode,
		    	   aASTNode** ppEndNode);
static aBool sMatchTypes(aSAST* pSAST,
			 aASTNode* pLeft,
			 aASTNode* pRight,
			 aASTNode** ppNewLeft,
			 aASTNode** ppNewRight,
			 aBool bLeftMutable,
			 aErr* pErr);
static aSymbolTableRef sSOIGetSymbolTable(aASTNode* pNode,
					  aASTNode** ppScopeNode);
/* static aErr sSOINodeSymDelete(void* pData, void* ref); */
static aErr sSOIFunctionCall(aSAST* pSAST,
		      aASTNode* pNode);
static aErr sSOICollectSymbols(aSAST* pSAST, 
			       aASTNode* pNode);
static aErr sSOIAddressReferences(aSAST* pSAST,
				  aASTNode* pNode);
static aErr sSOICleanUp(aSAST* pSAST,
			aASTNode* pNode);

static aErr sSOIMarkUsed(aSAST* pSAST, 
			 aASTNode* pNode);
static aErr sSOIMarkCalls(aSAST* pSAST,
	                  aASTNode* pNode);

static aErr sSOITranslationUnit(aSAST* pSAST, 
				aASTNode* pTranslationUnit);
static aErr sSOIExternalDeclaration(aSAST* pSAST, 
				    aASTNode* pExternalDeclaration);
static aErr sSOIFunctionDefinition(aSAST* pSAST, 
				   aASTNode* pFunction);
static aErr sSOIDeclarator(aSAST* pSAST,
    			   aASTNode* pDeclarator,
    			          aSFlags flags);
static aErr sSOIParameterList(aSAST* pSAST,
    			      aASTNode* pParameterList);
static aErr sSOIParameterDeclaration(aSAST* pSAST,
				     aASTNode* pParameterDeclaration);
static aErr sSOICompoundStatement(aSAST* pSAST,
    			          aASTNode* pCompoundStatement);
static aErr sSOIDeclarationList(aSAST* pSAST, 
				aASTNode* pDeclarationList);
static aErr sSOIDeclaration(aSAST* pSAST, 
			    aASTNode* pDeclaration);
static aErr sSOIInitDeclaratorList(aSAST* pSAST,
				   aASTNode* pInitDeclaratorList,
				   aSFlags flags);
static aErr sSOIInitDeclarator(aSAST* pSAST, 
			       aASTNode* pInitDeclarator,
			       aSFlags flags);
static aErr sSOIInitializer(aSAST* pSAST,
			    aASTNode* pInitializer,
			    aSFlags flags);
static aErr sSOIAssignmentExpression(aSAST* pSAST,
				     aASTNode* pAssignmentExpression,
				     aSFlags flags);
static aErr sSOIUnaryExpression(aSAST* pSAST,
				aASTNode* pUnaryExpression,
				aSFlags flags);
static aErr sSOIPostFixExpression(aSAST* pSAST,
				  aASTNode* pPostFixExpression,
				  aSFlags flags);
static aErr sSOIPrimaryExpression(aSAST* pSAST,
				  aASTNode* pPrimaryExpression,
				  aSFlags flags);
static aErr sSOIString(aSAST* pSAST,
		       aASTNode* pString,
		       aSFlags flags);
static aErr sSOIIdentifier(aSAST* pSAST,
			   aASTNode* pIdentifier,
			   aSFlags flags);
static aErr sSOIUnaryOperator(aSAST* pSAST,
			      aASTNode* pIdentifier,
			      aSFlags flags);
static aErr sSOICastExpression(aSAST* pSAST,
			       aASTNode* pIdentifier,
			       aSFlags flags);
static aErr sSOIShiftExpression(aSAST* pSAST,
			      	aASTNode* pShiftExpression,
				aSFlags flags);
static aErr sSOIRelationalExpression(aSAST* pSAST,
			      	     aASTNode* pRelationalExpression,
				     aSFlags flags);
static aErr sSOIEqualityExpression(aSAST* pSAST,
			      	   aASTNode* pEqualityExpression,
				   aSFlags flags);
static aErr sSOI2OpExpression(aSAST* pSAST,
			      	    aASTNode* p2OpExpression,
				    aSFlags flags);
static aErr sSOILogicalOpExpression(aSAST* pSAST,
			      	    aASTNode* pLogicalOpExpression,
				    aSFlags flags);
static aErr sSOIConditionalExpression(aSAST* pSAST,
			      	      aASTNode* pIdentifier,
				      aSFlags flags);

static aErr sSOIParameterList(aSAST* pSAST,
			      aASTNode* pParameterList);
static aErr sSOIStatementList(aSAST* pSAST,
			      aASTNode* pStatementList);
static aErr sSOIStatement(aSAST* pSAST,
			  aASTNode* pStatement);
static aErr sSOIAsmStatement(aSAST* pSAST,
			     aASTNode* pAsmStatement);
static aErr sSOIOpcode(aSAST* pSAST,
		       aASTNode* pOpcode);
static aErr sSOIJumpStatement(aSAST* pSAST,
			      aASTNode* pJumpStatement);
static aErr sSOIExpression(aSAST* pSAST,
			   aASTNode* pExpression,
			   aSFlags flags);
static aErr sSOIExpressionStatement(aSAST* pSAST,
				    aASTNode* pExpressionStatement);
static aErr sSOILabeledStatement(aSAST* pSAST,
				 aASTNode* pLabeledStatement);
static aErr sSOISelectionStatement(aSAST* pSAST,
				   aASTNode* pSelectionStatement);
static aErr sSOILoopStatement(aSAST* pSAST,
			      aASTNode* pLoopStatement);
static aErr sSOIExpressionProducer(aSAST* pSAST,
				   aASTNode* pNode,
				   aSFlags flags);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTypeCanPromote
 */

aBool sTypeCanPromote(aSFlags lValue, aSFlags rValue)
{
  if ((rValue & fByte) && (lValue & fShort))
    return aTrue;
  else
    return aFalse;

} /* sTypeCanPromote */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBuildAutoCast
 */

aErr sBuildAutoCast(aSAST* pSAST,
		    aSFlags type, 
		    aASTNode** ppNode,
		    aASTNode** ppEndNode)
{
  aErr soErr;
  aASTNode* pCast;
  aASTNode* pDeclSpec;

  aAssert(ppNode);

 /* create the splice */
  soErr = pSAST->createNode(pSAST, tCastExpression, &pCast);
  if (soErr == aErrNone) {
    soErr = pSAST->createNode(pSAST, tDeclarationSpecifier, 
    			      &pDeclSpec);
      pDeclSpec->flags |= type;
      if (soErr == aErrNone)
        soErr = pSAST->addNodeChild(pCast, pDeclSpec);
  }
  
  if (soErr == aErrNone) {
    *ppNode = pCast;
    *ppEndNode = pDeclSpec;
  }
  
  return soErr;

} /* sBuildAutoCast */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sMatchTypes
 */

aBool sMatchTypes(aSAST* pSAST,
		  aASTNode* pLeft,
		  aASTNode* pRight,
		  aASTNode** ppNewLeft,
		  aASTNode** ppNewRight,
		  aBool bLeftMutable,
		  aErr* pErr)
{
  aErr soErr = aErrNone;
  aBool bBuiltCast = aFalse;
  aBool bLPromote = aFalse;
  aBool bRPromote = aFalse;
  aSFlags lType;
  aSFlags rType;
  
  /* initialize the returns */
  aAssert(ppNewLeft);
  *ppNewLeft = NULL;
  aAssert(ppNewRight);
  *ppNewRight = NULL;

  /* get the types, stripping off the address */
  lType = (aSFlags)(pLeft->flags & fBaseTypeMask & (~fAddress));
  rType = (aSFlags)(pRight->flags & fBaseTypeMask & (~fAddress));

  /* if they don't match, see if the lesser can be promoted with
   * an implicit cast */
  if (lType != rType) {
    aASTNode* pCast = NULL;
    aASTNode* pDeclSpec = NULL;
  
    /* check if rType can be promoted to lType
     * then check if mutable lType can be promoted to rType */
    if (sTypeCanPromote(lType, rType) == aTrue) {
      bRPromote = aTrue;
    } else if ((bLeftMutable == aTrue)
               && (sTypeCanPromote(rType, lType) == aTrue)) {
      bLPromote = aTrue;
    } else {
      soErr = pSAST->outputErrProc(pSAST, pRight, aTE_INCOMPAT_TYPE);
    }
    
    /* if can promote, build the new cast node */
    /* at this time we can only promote bytes to shorts */
    if ((soErr == aErrNone)
        && ((bLPromote == aTrue) || (bRPromote == aTrue))) {

      soErr = sBuildAutoCast(pSAST, fShort, &pCast, &pDeclSpec);
      bBuiltCast = aTrue;

      /* assign the type being promoted to the
       * cast's decl spec child pointer */
      if (soErr == aErrNone) {
        if (bLPromote == aTrue) {
          pDeclSpec->pNext = pLeft;
        } else if (bRPromote == aTrue) {
          pDeclSpec->pNext = pRight;
        }
      }
    }
  
    /* call optimize on the cast expression to re-evalutate and handle
     * any optimization needed now that the cast is in place */
    if ((soErr == aErrNone) && (bBuiltCast == aTrue))
      soErr = sSOIExpressionProducer(pSAST, pCast,
    				     (aSFlags)(fGetType | fPushVal));
   
    /* assign newly constructed cast node to reply values */
    if ((soErr == aErrNone) && (bBuiltCast == aTrue)) {
      if (bLPromote == aTrue) {
        *ppNewLeft = pCast;
      } else if (bRPromote == aTrue) {
        *ppNewRight = pCast;
      }
    }
  }

  aAssert(pErr);
  *pErr = soErr;
  return bBuiltCast;
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIFunctionCall
 *
 * this is called with the identifier which is a data structure:
 *
 *     postfix
 *        |
 *       \|/
 *     primary --> arguments
 *        |
 *       \|/
 *    identifier 
 */

aErr sSOIFunctionCall(aSAST* pSAST,
		      aASTNode* pNode)
{
  aErr soErr = aErrNone;
  aASTNode* pCallPrev = NULL;
  aASTNode* pCall = NULL;
  aASTNode* pParam = NULL;
  int argument;

  aAssert (pNode->eType == tIdentifier);

  /* if this is an assembly call, we assume the user knows what they are 
   * doing */
  if (pNode->pParent->eType != tOpcode) {

    pCall = pNode->pParent;
    aAssert(pCall);
    aAssert(pCall->eType == tPrimaryExpression);
    pCall = pCall->pNext;
    aAssert(pCall);
    aAssert(pCall->eType == tArgumentExpressionList);
    pCall = pCall->pChildren;
    /* pCall now points to first calling parameter (or null) */

    pParam = pNode->pRef;
  }
  
  if (pParam) {
    aAssert(pParam->eType == tIdentifier);
    pParam = pParam->pNext;
    aAssert(pParam);
    aAssert(pParam->eType == tParameterList);
    pParam = pParam->pChildren;
    /* pParam now points to first parameter declaration (or null) */

    /* now, walk both lists and make sure the parameters are 
     * consistent in number and type, promoting types where possible */
    argument = 1;
    while ((soErr == aErrNone) && pCall && pParam) {
      aSFlags callType;
      aSFlags paramType = pParam->pChildren->flags & fBaseTypeMask;

      soErr = sSOIExpressionProducer(pSAST, pCall, (fGetType | fPushVal));
      
      if (soErr == aErrNone) {
        callType = pCall->flags & fBaseTypeMask;

        /* check type compatibility */
        if (callType != paramType) { 
          if (sTypeCanPromote(paramType, callType) == aTrue) {
            aASTNode* pCast;
            aASTNode* pDeclSpec;
	    soErr = sBuildAutoCast(pSAST, fShort, &pCast, &pDeclSpec);

            /* splice it in */
            if (soErr == aErrNone) {
              /* stick the old expression under the cast
               * the list looks like:
               *
               * pArgumentExpressionList
               *   |
               *  \|/ 
               * pCall1 --> pCall2 -->pCall3
               */
              pDeclSpec->pNext = pCall;
              pCast->pNext = pCall->pNext;
              pCall->pNext = NULL;
              if (pCallPrev)
                pCallPrev->pNext = pCast;
              else
                pCall->pParent->pChildren = pCast;
              /* changeling.... ee ee ee ee, recompute the expression
               * to allow optimization of the cast */
              soErr = sSOIExpressionProducer(pSAST, pCast,
    				  (aSFlags)(fGetType | fPushVal));
              /* fix the chain so loop will continue */
              pCall=pCast;    				  
            }
          } else {
            char line[100];
            char num[10];
            aStringCopy(line, aTE_ARGUMENT);
            aStringFromInt(num, argument);
            aStringCat(line, num);
            aStringCat(line, aTE_WRONGTYPE);
            soErr = pSAST->outputErrProc(pSAST, pCall, line);
          }
        }
      }
      argument++;
      pCallPrev = pCall;
      pCall = pCall->pNext;
      pParam = pParam->pNext;
    }
  
    /* now check for number of arguments */
    if (soErr == aErrNone) {
      if (pParam)
        soErr = pSAST->outputErrProc(pSAST, pNode, aTE_TOO_FEW_ARGS);
      else if (pCall)
        soErr = pSAST->outputErrProc(pSAST, pNode, aTE_TOO_MANY_ARGS);
    }
  }

  return soErr;

} /* sSOIFunctionCall */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOICollectSymbols
 *
 * This routine recursively visits the entire tree and both 
 * fabricates and populates the symbol tables for all scopes in 
 * the tree.
 */

aErr sSOICollectSymbols(aSAST* pSAST, 
			aASTNode* pNode)
{
  aErr soErr = aErrNone;
  aSymbolTableRef symbolTable;
  aASTNodeSym symData;
  
  aAssert(pSAST);
  aAssert(pNode);

  /* various node types are handled differently */
  switch (pNode->eType) {
 
  case tPostFixExpression:
    /* mark routine name as an address so it gets resolved on 
     * the address resolution pass */
    aAssert(pNode->pChildren);
    aAssert(pNode->pChildren->pChildren);
    if (pNode->flags & fPostFixRoutine)
      pNode->pChildren->pChildren->flags |= fAddress | fIdentRtnName;
    break;

  case tFunctionDefinition:
    /* build a symbol table for the function since functions have 
     * scope */
    aAssert(pNode->symbolTable == NULL);
    aSymbolTable_Create(pSAST->ioRef, &pNode->symbolTable, 
    			&soErr);
  
    /* go resolve the functions return type now for routine calls later */
    if (soErr == aErrNone) {
      aASTNode* pDeclarationSpecifier = pNode->pChildren;
      aAssert(pDeclarationSpecifier);
      aAssert(pDeclarationSpecifier->eType == tDeclarationSpecifier);
      soErr = aAST_OptDeclarationSpecifier((aAST*)pSAST, pDeclarationSpecifier);
      /* find the function's flags (return value) */
      if (soErr == aErrNone)
        pNode->flags |= (aSFlags)(pDeclarationSpecifier->flags & fTypeMask);
    }
    break;

  case tLabeledStatement:
    /* labels get added to the local scope's symbol table
     * these should really have their own private name-space */
    if (pNode->flags & fLabelGeneral) {
      symData.eType = stLabel;
      symData.pNode = pNode;

      /* add the label to the symbol table */
      symbolTable = sSOIGetSymbolTable(pNode, NULL);
      aAssert(symbolTable);
      aAssert(pNode->pToken);
      soErr = aASTNode_AddUniqueSymbol((aAST*)pSAST, 
      				       symbolTable, 
    				       pNode->pToken->v.identifier, 
    				       &symData, 
    				       pNode);
    }
    break;
  
  case tDeclarator: {
    /* declarators always get added to the symbol table */
    aASTNode* pIdentifier;
    pIdentifier = pNode->pChildren;
  
    aAssert(pIdentifier->eType == tIdentifier);
 
    symData.eType = stSymbol;
    symData.pNode = pIdentifier;

    /* function declarators store the name declarator in 
     * the global symbol table
     */
    if (pNode->pParent->eType == tFunctionDefinition) {
      
      /* walk up to the root node */
      aASTNode* pTemp = pNode;
      while (pTemp) {
        if (pTemp->pParent == NULL)
          symbolTable = pTemp->symbolTable;
        pTemp = pTemp->pParent;
      } /* while */

    /* otherwise, just get the table from the current scope */
    } else
      symbolTable = sSOIGetSymbolTable(pNode, NULL);

    /* add the declarator to the symbol table */
    soErr = aASTNode_AddUniqueSymbol((aAST*)pSAST, 
			             symbolTable, 
    				     pIdentifier->pToken->v.identifier, 
    				     &symData, 
    			  	     pIdentifier);
    }
    break;
  
  default:
    break;

  } /* switch */

  /* then visit children */
  if ((soErr == aErrNone) &&
      (pNode->pChildren != NULL))
    soErr = sSOICollectSymbols(pSAST, pNode->pChildren);

  /* and finally siblings */
  if ((soErr == aErrNone) &&
      (pNode->pNext != NULL))
    soErr = sSOICollectSymbols(pSAST, pNode->pNext);

  return soErr;

} /* sSOICollectSymbols */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIAddressReferences
 */

aErr sSOIAddressReferences(aSAST* pSAST,
			   aASTNode* pNode)
{
  aErr soErr = aErrNone;

  aAssert(pSAST);
  aAssert(pNode);

  /* try to find the address in the symbol table */
  if (pNode->flags & fAddress) {
    void* vpSymData;
    aSymbolTableRef tableRef;
    aErr symErr;
    char line[100];

    /* need to find the root symbol table for routine lookups */
    if (pNode->flags & fIdentRtnName)
      tableRef = pSAST->pTree->symbolTable;
    else
      tableRef = sSOIGetSymbolTable(pNode, NULL);

    aAssert(tableRef);
    aAssert(pNode->pToken);
    
    /* look it up in the table */
    aSymbolTable_Find(pSAST->ioRef, tableRef, 
    		      pNode->pToken->v.identifier, 
      		      &vpSymData, &symErr);
    switch (symErr) {
    case aErrNone:
      pNode->pRef = ((aASTNodeSym*)vpSymData)->pNode;
      if (((aASTNodeSym*)vpSymData)->eType == stLabel)
        pNode->flags |= fConstant;
      else if (pNode->flags & fIdentRtnName) {
        /* get the type of the function call from the return 
         * value of the called routine */
        aAssert(pNode->pRef); /* identifier */
        aAssert(pNode->pRef->pParent); /* declarator */
        aAssert(pNode->pRef->pParent->pParent); /* function */
        pNode->flags |= (aSFlags)(pNode->pRef->pParent->pParent->flags & fTypeMask);
      }
      break;
    case aErrNotFound:
      aStringCopy(line, aTE_UNDEFINED_SYMBOLx);
      aStringCat(line, pNode->pToken->v.identifier);
      soErr = pSAST->outputErrProc(pSAST, pNode, line);
      break;
    default:
      soErr = symErr;
      break;
    } /* switch */
  }

  /* then visit children */
  if ((soErr == aErrNone) &&
      (pNode->pChildren != NULL))
    soErr = sSOIAddressReferences(pSAST, pNode->pChildren);

  /* and finally siblings */
  if ((soErr == aErrNone) &&
      (pNode->pNext != NULL))
    soErr = sSOIAddressReferences(pSAST, pNode->pNext);

  return soErr;

} /* sSOIAddressReferences */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOICleanUp
 */

aErr sSOICleanUp(aSAST* pSAST,
	         aASTNode* pNode)
{
  aErr soErr = aErrNone;

  aAssert(pSAST);
  aAssert(pNode);

  if (pNode->symbolTable) {
    aSymbolTable_Destroy(pSAST->ioRef, pNode->symbolTable, &soErr);
    pNode->symbolTable = NULL;
  }

  /* then visit children */
  if ((soErr == aErrNone) &&
      (pNode->pChildren != NULL))
    soErr = sSOICleanUp(pSAST, pNode->pChildren);

  /* and finally siblings */
  if ((soErr == aErrNone) &&
      (pNode->pNext != NULL))
    soErr = sSOICleanUp(pSAST, pNode->pNext);

  return soErr;

} /* sSOICleanUp */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIMarkUsed
 */

aErr sSOIMarkUsed(aSAST* pSAST,
	         aASTNode* pNode)
{
  aErr soErr = aErrNone;

  aAssert(pSAST);

  if (!pNode)
    return soErr;

  aAssert(pNode->eType == tFunctionDefinition);

  /* set this routine to used */
  pNode->flags |= fFunctionUsed;

  soErr = sSOIMarkCalls(pSAST,
  			pNode->pChildren);

  return soErr;

} /* sSOIMarkUsed */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIMarkCalls
 */

aErr sSOIMarkCalls(aSAST* pSAST,
	           aASTNode* pNode)
{
  aErr soErr = aErrNone;

  aAssert(pSAST);
  aAssert(pNode);

  /* see if we are at a routine call */  
  if ((pNode->flags & fAddress)
      && (pNode->flags & fIdentRtnName)
      && (pNode->pRef)) {
    aASTNode* pCalled;
    aAssert(pNode->pRef->pParent);
    aAssert(pNode->pRef->pParent->pParent);

    pCalled = pNode->pRef->pParent->pParent;

    aAssert(pCalled->eType == tFunctionDefinition);

    /* watch for recursion skipping routines already marked */
    if (!(pCalled->flags & fFunctionUsed))
      soErr = sSOIMarkUsed(pSAST, pCalled);
  }

  /* then visit children */
  if ((soErr == aErrNone) &&
      (pNode->pChildren != NULL))
    soErr = sSOIMarkCalls(pSAST, pNode->pChildren);

  /* and finally siblings */
  if ((soErr == aErrNone) &&
      (pNode->pNext != NULL))
    soErr = sSOIMarkCalls(pSAST, pNode->pNext);

  return soErr;

} /* sSOIMarkCalls */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIGetSymbolTable
 * 
 * looks for the first (deepest) scoping symbol table
 */

aSymbolTableRef sSOIGetSymbolTable(aASTNode* pNode,
				  aASTNode** ppScopeNode)
{
  aASTNode* pTemp = pNode;
  
  while (pTemp && (pTemp->symbolTable == NULL))
    pTemp = pTemp->pParent;
  
  if (pTemp) {
    if (ppScopeNode != NULL)
      *ppScopeNode = pTemp;
    return pTemp->symbolTable;
  } else
    return NULL;

} /* sSOIGetSymbolTable */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOITranslationUnit
 */

aErr sSOITranslationUnit(aSAST* pSAST, 
			 aASTNode* pTranslationUnit)
{
  aErr soErr = aErrNone;
  aASTNode* pExternalDeclaration;
  
  aAssert(pSAST);
  aAssert(pTranslationUnit->eType == tTranslationUnit);

  /* walk through and purify all the external declarations */
  pExternalDeclaration = pTranslationUnit->pChildren;
  while ((soErr == aErrNone)
         && (pExternalDeclaration != NULL)) {
    soErr = sSOIExternalDeclaration(pSAST, pExternalDeclaration);
    pExternalDeclaration = pExternalDeclaration->pNext;
  }

  return soErr;

} /* sSOITranslationUnit */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIExternalDeclaration
 */

aErr sSOIExternalDeclaration(aSAST* pSAST, 
			     aASTNode* pExternalDeclaration)
{
  aErr soErr = aErrNone;
  aASTNode* pChild;
  
  aAssert(pSAST);
  aAssert(pExternalDeclaration->eType == tExternalDeclaration);
  
  /* snag the flags from the flags specifier */
  pChild = pExternalDeclaration->pChildren;
  if (pChild != NULL) {

    switch (pChild->eType) {

    case tFunctionDefinition:
      soErr = sSOIFunctionDefinition(pSAST, pChild);
      break;

    case tDeclaration:
      soErr = sSOIDeclaration(pSAST, pChild);
      break;
  
    default:
      aAssert(0); /* should we get here ? */
      break;

    } /* switch */

  } /* while */
  
  return soErr;

} /* sSOIExternalDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIFunctionDefinition
 */

aErr sSOIFunctionDefinition(aSAST* pSAST, 
			    aASTNode* pFunction)
{
  aErr soErr = aErrNone;
  aASTNode* pDeclarationSpecifier;
  aASTNode* pDeclarator;
  aASTNode* pCompoundStatement;

  aAssert(pSAST);
  aAssert(pFunction);
  
  pSAST->pCurrentFunction = pFunction;

  /* get hold of the declaration specifier */
  if (soErr == aErrNone) {
    pDeclarationSpecifier = pFunction->pChildren;
    aAssert(pDeclarationSpecifier);
    aAssert(pDeclarationSpecifier->eType == tDeclarationSpecifier);
    /* return type is already handled in the collection of symbols */
  }

  /* now, handle the declarator */
  if (soErr == aErrNone) {
    pDeclarator = pDeclarationSpecifier->pNext;
    aAssert(pDeclarator);
    aAssert(pDeclarator->eType == tDeclarator);
    soErr = sSOIDeclarator(pSAST, pDeclarator, 
    			   (aSFlags)(pFunction->flags & fTypeMask));
  }

  /* now, handle the compound statement */
  if (soErr == aErrNone) {
    pCompoundStatement = pDeclarator->pNext;
    aAssert(pCompoundStatement);
    aAssert(pCompoundStatement->eType == tCompoundStatement);
    soErr = sSOICompoundStatement(pSAST, pCompoundStatement);
  }

  /* ensure a return value of proper type */
  if (!(pDeclarationSpecifier->flags & fVoid)) {
    if ((pSAST->pLastStatement == NULL)
        || (pSAST->pLastStatement->pChildren->eType != tJumpStatement)
        || (pSAST->pLastStatement->pChildren->pChildren == NULL))
      soErr = pSAST->outputErrProc(pSAST, pDeclarator, 
      				  aTE_RETURN_REQUIRED);
    else {
      aASTNode* pReturn = pSAST->pLastStatement->pChildren;
      aSFlags rtnType = pDeclarationSpecifier->flags & fBaseTypeMask;
      aSFlags retType = pReturn->pChildren->flags & (fBaseTypeMask & ~fAddress);

      /* check type compatibility */
      if (rtnType != retType) { 
        if (sTypeCanPromote(rtnType, retType) == aTrue) {
          aASTNode* pCast;
          aASTNode* pDeclSpec;
	  soErr = sBuildAutoCast(pSAST, fShort, &pCast, &pDeclSpec);

          /* splice it in */
          if (soErr == aErrNone) {
            /* stick the old expression under the cast */
            pDeclSpec->pNext = pReturn->pChildren;
            pReturn->pChildren = pCast;
            /* changeling.... ee ee ee ee, recompute the expression
             * to allow optimization of the cast */
            soErr = sSOIExpressionProducer(pSAST, pCast,
    				  (aSFlags)(fGetType | fPushVal));
          }
        } else {
          soErr = pSAST->outputErrProc(pSAST, pReturn, 
				      aTE_INCOMPAT_RETTYPE);
        }
      }
    }
  }

  pSAST->pCurrentFunction = NULL;

  return soErr;

} /* sSOIFunctionDefinition */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIDeclarator
 *
 * declarators are in the symbol table
 */

aErr sSOIDeclarator(aSAST* pSAST,
    		    aASTNode* pDeclarator,
    		    aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pIdentifier;

  aAssert(pSAST);
  aAssert(pDeclarator);
  aAssert(pDeclarator->eType == tDeclarator);

  pDeclarator->flags |= flags;

  /* declarators should always start with an identifier */
  pIdentifier = pDeclarator->pChildren;
  aAssert(pIdentifier);
  aAssert(pIdentifier->eType == tIdentifier);
  aAssert(pIdentifier->pToken);
  
  pIdentifier->flags |= flags;

  /* leave a bread crumb for the code generator telling them
   * this identifier is a local variable declaration */
  if (pSAST->bInDeclarationList == aTrue)
    pIdentifier->flags |= fIdentLocalVar;

  /* now check for a parameter list */
  if ((soErr == aErrNone)
      && (pIdentifier->pNext != NULL)) {
    aASTNode* pParameterList = pIdentifier->pNext;
    soErr = sSOIParameterList(pSAST, pParameterList);
  }

  return soErr;

} /* sSOIDeclarator */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIParameterList
 */

aErr sSOIParameterList(aSAST* pSAST,
    	                aASTNode* pParameterList)
{
  aErr soErr = aErrNone;
  aASTNode* pParameterDeclaration;
  
  aAssert(pSAST);
  aAssert(pParameterList->eType == tParameterList);
  
  pParameterDeclaration = pParameterList->pChildren;
  
  while ((soErr == aErrNone) 
         && (pParameterDeclaration != NULL)) {
    soErr = sSOIParameterDeclaration(pSAST, pParameterDeclaration);
    pParameterDeclaration = pParameterDeclaration->pNext;
  }

  return soErr;

} /* sSOIParameterList */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIParameterDeclaration
 */

aErr sSOIParameterDeclaration(aSAST* pSAST,
    			      aASTNode* pParameterDeclaration)
{
  aErr soErr = aErrNone;
  aASTNode* pDeclarationSpecifier;
  
  aAssert(pSAST);
  aAssert(pParameterDeclaration->eType == tParameterDeclaration);
  
  /* first, handle the required declaration specifier */
  pDeclarationSpecifier = pParameterDeclaration->pChildren;
  soErr = aAST_OptDeclarationSpecifier((aAST*)pSAST, pDeclarationSpecifier);

  /* then, get the declarator */
  if (soErr == aErrNone) {
    aASTNode* pDeclarator = pDeclarationSpecifier->pNext;
    soErr = sSOIDeclarator(pSAST, pDeclarator, 
    			   (aSFlags)(pDeclarationSpecifier->flags & fTypeMask));

    /* routine parameters are implicitly initialized so mark them so */
    aAssert(pDeclarator->pChildren);
    aAssert(pDeclarator->pChildren->eType == tIdentifier);
    pDeclarator->pChildren->flags |= fIdentInited;
  }

  return soErr;

} /* sSOIParameterDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOICompoundStatement
 */

aErr sSOICompoundStatement(aSAST* pSAST,
			   aASTNode* pCompoundStatement)
{
  aErr soErr = aErrNone;
  aASTNode* pTemp;
  
  aAssert(pSAST);
  aAssert(pCompoundStatement);
  aAssert(pCompoundStatement->eType == tCompoundStatement);

  /* check for and handle a declaration list if present */
  pTemp = pCompoundStatement->pChildren;
  
  if (pTemp) {
    if (pTemp->eType == tDeclarationList) {
      soErr = sSOIDeclarationList(pSAST, pTemp);
      if (soErr == aErrNone)
        pTemp = pTemp->pNext;
    }

    /* next, handle the statement list */
    if (soErr == aErrNone) {
      aAssert(pTemp->eType == tStatementList);
      soErr = sSOIStatementList(pSAST, pTemp);
    }
  }

  return soErr;

} /* sSOICompoundStatement */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIDeclarationList
 */

aErr sSOIDeclarationList(aSAST* pSAST, 
			    aASTNode* pDeclarationList)
{
  aErr soErr = aErrNone;
  aASTNode* pDeclaration;
  
  aAssert(pDeclarationList);
  aAssert(pDeclarationList->eType == tDeclarationList);

  pSAST->bInDeclarationList = aTrue;
  pDeclaration = pDeclarationList->pChildren;
  while ((pDeclaration != NULL) 
         && (soErr == aErrNone)) {
    soErr = sSOIDeclaration(pSAST, pDeclaration);
    pDeclaration = pDeclaration->pNext;
  }
  pSAST->bInDeclarationList = aFalse;

  return soErr;

} /* sSOIDeclarationList */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIDeclaration
 */

aErr sSOIDeclaration(aSAST* pSAST, 
		     aASTNode* pDeclaration)
{
  aErr soErr = aErrNone;
  aASTNode* pDeclarationSpecifier;
  aASTNode* pInitDeclaratorList;

  aAssert(pSAST);
  aAssert(pDeclaration);
  aAssert(pDeclaration->eType == tDeclaration);

  /* declaration specifier always present */
  pDeclarationSpecifier = pDeclaration->pChildren;
  aAssert(pDeclarationSpecifier);
  aAssert(pDeclarationSpecifier->eType == tDeclarationSpecifier);

  /* see if we are global and push down flags if we are */
  if (pDeclaration->pParent->eType == tExternalDeclaration) {
    pDeclaration->flags |= fGlobal;
    pDeclarationSpecifier->flags |= fGlobal;
  }

  soErr = aAST_OptDeclarationSpecifier((aAST*)pSAST, pDeclarationSpecifier);

  if (soErr == aErrNone) {
    pInitDeclaratorList = pDeclarationSpecifier->pNext;
    aAssert(pInitDeclaratorList);
    aAssert(pInitDeclaratorList->eType == tInitDeclaratorList);
    soErr = sSOIInitDeclaratorList(pSAST, pInitDeclaratorList,
    				   pDeclarationSpecifier->flags);
  }

  return soErr;

} /* sSOIDeclaration */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIInitDeclaratorList 
 */

aErr sSOIInitDeclaratorList(aSAST* pSAST,
			    aASTNode* pInitDeclaratorList,
			    aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pInitDeclarator;
  
  aAssert(pInitDeclaratorList);
  aAssert(pInitDeclaratorList->eType == tInitDeclaratorList);

  /* set the flags */
  pInitDeclaratorList->flags |= flags;

  pInitDeclarator = pInitDeclaratorList->pChildren;
  while ((pInitDeclarator != NULL) 
         && (soErr == aErrNone)) {
    soErr = sSOIInitDeclarator(pSAST, pInitDeclarator, flags);
    pInitDeclarator = pInitDeclarator->pNext;
  }

  return soErr;

} /* sSOIInitDeclaratorList */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIInitDeclarator
 */

aErr sSOIInitDeclarator(aSAST* pSAST, 
			aASTNode* pInitDeclarator,
			aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pDeclarator;
  aASTNode* pInitializer;
  
  aAssert(pInitDeclarator);
  aAssert(pInitDeclarator->eType == tInitDeclarator);

  /* set the flags */
  pInitDeclarator->flags |= flags;

  /* init declarators always have a declarator as the first
   * element followed by an optional initializer */
  pDeclarator = pInitDeclarator->pChildren;
  pInitializer = pDeclarator->pNext;

  /* handle the required declarator */
  soErr = sSOIDeclarator(pSAST, pDeclarator, flags);

  /* the optional initializer */
  if ((soErr == aErrNone)
      && (pInitializer != NULL))
    soErr = sSOIInitializer(pSAST, pInitializer, flags);

  return soErr;

} /* sSOIInitDeclarator */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIInitializer
 */

aErr sSOIInitializer(aSAST* pSAST,
		     aASTNode* pInitializer,
		     aSFlags flags)
{
  aErr soErr = aErrNone;
  
  aAssert(pInitializer);
  aAssert(pInitializer->eType == tInitializer);
  aAssert(pInitializer->pChildren != NULL);

  pInitializer->flags |= (aSFlags)(flags | fPushVal);

  soErr = sSOIExpressionProducer(pSAST, pInitializer->pChildren,
    				 (aSFlags)(flags | fPushVal));

  return soErr;

} /* sSOIInitializer */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIAssignmentExpression
 */

aErr sSOIAssignmentExpression(aSAST* pSAST,
			      aASTNode* pAssignmentExpression,
			      aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pLValue;
  aASTNode* pAssignmentOperator;
  aASTNode* pRValue = NULL;
  aASTNode* pNewLeft = NULL;
  aASTNode* pNewRight = NULL;

  aAssert(pAssignmentExpression);
  aAssert(pAssignmentExpression->eType == tAssignmentExpression);

  pAssignmentExpression->flags |= flags;

  /* assignment expressions always start with a unary */
  pLValue = pAssignmentExpression->pChildren;
  aAssert(pLValue);

  /* assignment operators are optional */
  pAssignmentOperator = pLValue->pNext;

  /* set flag for storage if assigning */
  if (pAssignmentOperator != NULL) {
    soErr = sSOIExpressionProducer(pSAST, pLValue, 
    				   (aSFlags)(flags | fGetType));
    pRValue = pAssignmentOperator->pNext;
    aAssert(pRValue);
  } else {
    soErr = sSOIExpressionProducer(pSAST, pLValue, flags);
  }

  if (soErr == aErrNone) {
    if (pRValue != NULL) {

      soErr = sSOIExpressionProducer(pSAST, pRValue,
    				     (aSFlags)(fGetType | fPushVal));

      /* changeling.... ee ee ee ee */

      /* make sure the types match (Left is NOT mutable) */
      if (soErr == aErrNone) {
        if (sMatchTypes(pSAST, pLValue, pRValue, &pNewLeft, &pNewRight,
                        aFalse, &soErr) == aTrue) {
          if ((soErr == aErrNone) && pNewRight)
            pAssignmentOperator->pNext = pNewRight;
        }
      }

    } else {
      soErr = aAST_PushUpFlags(pAssignmentExpression, pLValue);
    }
  }

  return soErr;

} /* sSOIAssignmentExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIUnaryExpression
 */

aErr sSOIUnaryExpression(aSAST* pSAST,
			 aASTNode* pUnaryExpression,
			 aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pChildren;

  aAssert(pUnaryExpression);
  aAssert(pUnaryExpression->eType == tUnaryExpression);

  pUnaryExpression->flags |= flags;

  pChildren = pUnaryExpression->pChildren;
  aAssert(pChildren);

  soErr = sSOIExpressionProducer(pSAST, pChildren, flags);

  /* make sure the expression is mutable */
  if ((soErr == aErrNone)
      && pUnaryExpression->flags & (fUnaryPreInc | fUnaryPreInc)) {
    if (pChildren->flags & fConstant) {
      soErr = pSAST->outputErrProc(pSAST, pUnaryExpression, 
      				  aTE_ILLEGAL_CONST_OP);
    }
  }

  if (soErr == aErrNone)
    soErr = aAST_PushUpFlags(pUnaryExpression, pChildren);

  return soErr;

} /* sSOIUnaryExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIPostFixExpression
 */

aErr sSOIPostFixExpression(aSAST* pSAST,
			   aASTNode* pPostFixExpression,
			   aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pChild;
  
  aAssert(pPostFixExpression);
  aAssert(pPostFixExpression->eType == tPostFixExpression);

  pPostFixExpression->flags |= flags;

  pChild = pPostFixExpression->pChildren;
  aAssert(pChild);

  /* mark routine calls as addresses so the symbols are not sought
   * in this pass */
  if (pPostFixExpression->flags & fPostFixRoutine) {
    aAssert(pChild->pChildren);
    /* mark the routine name identifier as an address */
    pChild->pChildren->flags |= fIdentRtnName;
  }

  soErr = sSOIPrimaryExpression(pSAST, pChild, 
  				(aSFlags)(flags | fGetType));

  /* handle postincrement and postdecrement */
  if ((soErr == aErrNone)
      && (pPostFixExpression->flags & (fPostFixInc | fPostFixDec))) {
    if (pChild->flags & fConstant)
      soErr = pSAST->outputErrProc(pSAST, pChild, aTE_ILLEGAL_CONST_OP);
      
    /* go find the reference to increment or decrement */
    else {
      aASTNode* pTemp = pChild;
      while (pTemp && pTemp->eType != tIdentifier)
        pTemp = pTemp->pChildren;
      aAssert(pTemp);
      aAssert(pTemp->pRef);
      pPostFixExpression->pRef = pTemp->pRef;
    }
  }

  if ((soErr == aErrNone)
      && (pPostFixExpression->flags & fPostFixRoutine)) {
#ifdef aDEBUG
    aASTNode* pArguments = NULL;
#endif /* aDEBUG */

    /* only identifiers should be routine names */
    if (pChild->pChildren->eType != tIdentifier)
      soErr = pSAST->outputErrProc(pSAST, pChild->pChildren, 
      				  aTE_INVALID_ROUTINE_CALL);

    /* ensure argument list (could be null) */
#ifdef aDEBUG
    if (pArguments) {	
      aAssert(pArguments->eType == tArgumentExpressionList);
    }
#endif
  }
 
  if (soErr == aErrNone)
    soErr = aAST_PushUpFlags(pPostFixExpression, pChild);

  return soErr;

} /* sSOIPostFixExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIPrimaryExpression
 */

aErr sSOIPrimaryExpression(aSAST* pSAST,
			   aASTNode* pPrimaryExpression,
			   aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pChildren;

  aAssert(pPrimaryExpression);
  aAssert(pPrimaryExpression->eType == tPrimaryExpression);
  
  pPrimaryExpression->flags |= flags;

  pChildren = pPrimaryExpression->pChildren;
  aAssert(pChildren);

  switch (pChildren->eType) {

  case tConstant:
    soErr = aAST_OptConstant((aAST*)pSAST, pChildren, flags);
    break;

  case tString:
    soErr = sSOIString(pSAST, pChildren, flags);
    break;

  case tIdentifier:
    soErr = sSOIIdentifier(pSAST, pChildren, flags);
    break;
    
  case tExpression:
    soErr = sSOIExpression(pSAST, pChildren, flags);
    break;
  
  default:
    aAssert(0); /* should we get here ? */
    break;

  } /* switch */
  
  if (soErr == aErrNone)
    soErr = aAST_PushUpFlags(pPrimaryExpression, pChildren);

  return soErr;

} /* sSOIPrimaryExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIString
 */

aErr sSOIString(aSAST* pSAST,
		aASTNode* pString,
		aSFlags flags)
{
  aErr soErr = aErrNone;

  aAssert(pString);
  aAssert(pString->eType == tString);
  
  pString->flags |= flags;

  return soErr;

} /* sSOIString */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIIdentifier
 */

aErr sSOIIdentifier(aSAST* pSAST,
		    aASTNode* pIdentifier,
		    aSFlags flags)
{
  aErr soErr = aErrNone;
  aErr symErr;
  aASTNode* pScopeNode = NULL;
  aSymbolTableRef symbolTable;
  void* vpSymData;

  aAssert(pIdentifier);
  aAssert(pIdentifier->pParent);
  
  pIdentifier->flags |= flags;

  /* addresses and routines are handled specially */
  if (pIdentifier->flags & fIdentRtnName) {
    soErr = sSOIFunctionCall(pSAST, pIdentifier);

  } else {

    /* try to find the symbol by looking up the scope stack */
    symbolTable = sSOIGetSymbolTable(pIdentifier, &pScopeNode);
    symErr = aErrNotFound;
    while ((symErr == aErrNotFound)
           && (symbolTable != NULL)) {
      aSymbolTable_Find(pSAST->ioRef, symbolTable,
      			pIdentifier->pToken->v.identifier,
      			&vpSymData, &symErr);
      if (symErr == aErrNotFound)
        symbolTable = sSOIGetSymbolTable(pScopeNode->pParent, 
        			         &pScopeNode);
    } /* while */

    /* error if we couldn't find the symbol */  
    if (symErr == aErrNotFound) {
      char line[100];
      aStringCopy(line, aTE_UNDEFINED_SYMx);
      aStringCat(line, pIdentifier->pToken->v.identifier);
      soErr = pSAST->outputErrProc(pSAST, pIdentifier, line);

    /* otherwise, cash the reference and check for type compatibility */
    } else if (symErr == aErrNone) {
/*
      aSFlags identType = (aSFlags)(flags & fTypeMask);
      aSFlags refType = (aSFlags)(pSymData->pNode->flags & fTypeMask);
 */

/* ??? */
/* should there be a comparison */
/* between identType and refType? */

      switch (((aASTNodeSym*)vpSymData)->eType) {

      case stSymbol:
        /* cash the reference and set the type */
        pIdentifier->pRef = ((aASTNodeSym*)vpSymData)->pNode;
        pIdentifier->flags |= (aSFlags)(((aASTNodeSym*)vpSymData)->pNode->flags & fTypeMask);
        break;

  
      default:
        break;

      } /* switch pSymData->eType */
    }
  }

  return soErr;

} /* sSOIIdentifier */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIUnaryOperator 
 */

aErr sSOIUnaryOperator(aSAST* pSAST,
		       aASTNode* pUnaryOperator,
		       aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pCastExpression;
  
  aAssert(pUnaryOperator);
  aAssert(pUnaryOperator->eType == tUnaryOperator);

  pUnaryOperator->flags |= flags;

  pCastExpression = pUnaryOperator->pNext;

  if (pUnaryOperator->flags & fUnaryOpBang) {
    pUnaryOperator->flags &= ~fTypeMask;
    pUnaryOperator->flags |= fByte;
    flags &= ~fTypeMask;
    flags |= fGetType;
  }

  soErr = sSOIExpressionProducer(pSAST, pCastExpression, flags);

  if (soErr == aErrNone) {

    /* bang! always converts to tBYTE so request the type 
     * from the operand for the code generator to perform
     * conversions, otherwise, just pass up the type */
    if ((pCastExpression->flags & fGetType)
        && !(pUnaryOperator->flags & fUnaryOpBang)) {
      pUnaryOperator->flags |= 
      		(aSFlags)(pCastExpression->flags & fTypeMask);
    }

    /* constants can be pre-computed */
    if (pCastExpression->flags & fConstant) {
      pUnaryOperator->flags |= fConstant;

      /* - */
      if (pUnaryOperator->flags & fUnaryOpMinus) {
        if (pCastExpression->flags & fByte)
          pUnaryOperator->v.byteVal = (tBYTE)-pCastExpression->v.byteVal;
        if (pCastExpression->flags & fShort)
          pUnaryOperator->v.shortVal = (tSHORT)-pCastExpression->v.shortVal;

      /* + */
      } else if (pUnaryOperator->flags & fUnaryOpPlus) {
        if (pCastExpression->flags & fByte)
          pUnaryOperator->v.byteVal = pCastExpression->v.byteVal;
        if (pCastExpression->flags & fShort)
          pUnaryOperator->v.shortVal = pCastExpression->v.shortVal;

      /* ! */
      } else if (pUnaryOperator->flags & fUnaryOpBang) {
        if (pCastExpression->flags & fByte) {
          if (pCastExpression->v.byteVal)
            pUnaryOperator->v.byteVal = 0;
          else
            pUnaryOperator->v.byteVal = 1;
        } else if (pCastExpression->flags & fShort) {
          if (pCastExpression->v.shortVal)
            pUnaryOperator->v.byteVal = 0;
          else
            pUnaryOperator->v.byteVal = 1;
        }

      /* ~ */
      } else if (pUnaryOperator->flags & fUnaryOpTilde) {
        if (pCastExpression->flags & fByte)
          pUnaryOperator->v.byteVal = (tBYTE)~pCastExpression->v.byteVal;
        if (pCastExpression->flags & fShort)
          pUnaryOperator->v.shortVal = (tSHORT)~pCastExpression->v.shortVal;
      
      }
    }
  }

  return soErr;

} /* sSOIUnaryOperator */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOICastExpression 
 */

aErr sSOICastExpression(aSAST* pSAST,
			aASTNode* pCastExpression,
			aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pType;
  aASTNode* pChild;
  
  aAssert(pCastExpression);
  aAssert(pCastExpression->eType == tCastExpression);

  pCastExpression->flags |= flags;

  pType = pCastExpression->pChildren;
  aAssert(pType);
  aAssert(pType->eType == tDeclarationSpecifier);
  soErr = aAST_OptDeclarationSpecifier((aAST*)pSAST, pType);

  if ((soErr == aErrNone)
      && (pCastExpression->flags & fGetType))
    pCastExpression->flags |= pType->flags & (fTypeMask | fUnsigned);

  if (soErr == aErrNone) {
    pChild = pType->pNext;
    aAssert(pChild);

    /* purify the sub-type and get its type */
    soErr = sSOIExpressionProducer(pSAST, pChild, 
  				   (flags & ~fTypeMask) | fGetType);
  }

  /* just do the cast here if constant and there is actually 
   * a conversion */
  if ((soErr == aErrNone)
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
      soErr = pSAST->outputErrProc(pSAST, pCastExpression, 
      				  aTE_INVALID_CAST);
  }

  return soErr;

} /* sSOICastExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIShiftExpression 
 *
 * used for << and >> operations
 */

aErr sSOIShiftExpression(aSAST* pSAST,
			 aASTNode* pShiftExpression,
			 aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pLeft;
  aASTNode* pRight;
  aSFlags childFlags = (aSFlags)((flags & ~fTypeMask) | fGetType);

  aAssert(pSAST);
  aAssert(pShiftExpression);

  /* don't send down the type, we want to find it out */
  pShiftExpression->flags |= (aSFlags)(flags & ~fTypeMask);

  pLeft = pShiftExpression->pChildren;
  aAssert(pLeft);
  pRight = pLeft->pNext;
  aAssert(pRight);
  
  /* purify the left side of the operator */
  soErr = sSOIExpressionProducer(pSAST, pLeft, childFlags);

  /* purify the right side of the operator */
  if (soErr == aErrNone)
    soErr = sSOIExpressionProducer(pSAST, pRight, childFlags);

  /* precompute if constant */
  if ((soErr == aErrNone)
      && (pLeft->flags & fConstant)
      && (pRight->flags & fConstant)) {
    tSHORT amount = 0;

    /* first, get the shift amount */
    if (pRight->flags & fByte) {
      amount = pRight->v.byteVal;
    } else if (pRight->flags & fShort) {
      amount = pRight->v.shortVal;
    } else {
      soErr = pSAST->outputErrProc(pSAST, pRight, 
      				  aTE_INVALID_SHIFT_AMOUNT);
    }

    /* now, do the shift */
    if (amount >= 0) {
      if (pShiftExpression->flags & fShiftLeft) { 
        if (pLeft->flags & fByte) {
          pShiftExpression->v.byteVal = 
        		(tBYTE)(pLeft->v.byteVal << amount);
        } else if (pLeft->flags & fShort) {
          pShiftExpression->v.shortVal = 
        		(tSHORT)(pLeft->v.shortVal << amount);
        }
      } else if (pShiftExpression->flags & fShiftRight) { 
        if (pLeft->flags & fByte) {
          pShiftExpression->v.byteVal = 
        		(tBYTE)(pLeft->v.byteVal >> amount);
        } else if (pLeft->flags & fShort) {
          pShiftExpression->v.shortVal = 
        		(tSHORT)(pLeft->v.shortVal >> amount);
        }
      }
    }
    pShiftExpression->flags |= fConstant;

  }

  /* set the type to the left side's type */
  if (soErr == aErrNone)
    pShiftExpression->flags |= (aSFlags)(pLeft->flags & fTypeMask);

  return soErr;
  
} /* sSOIShiftExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIRelationalExpression 
 *
 * used for <, >, <=, and >= operations
 */

aErr sSOIRelationalExpression(aSAST* pSAST,
			      aASTNode* pRelationalExpression,
			      aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pLeft;
  aASTNode* pRight;
  aASTNode* pNewLeft = NULL;
  aASTNode* pNewRight = NULL;
  aSFlags lType, rType;
  aSFlags childFlags = (aSFlags)((flags & ~fTypeMask) | fGetType);
  aSFlags op;

  aAssert(pSAST);
  aAssert(pRelationalExpression);

  /* don't send down the type, we want to find it out */
  pRelationalExpression->flags |= (aSFlags)(flags & ~fTypeMask);

  pLeft = pRelationalExpression->pChildren;
  aAssert(pLeft);
  pRight = pLeft->pNext;
  aAssert(pRight);
  
  /* purify the left side of the operator */
  soErr = sSOIExpressionProducer(pSAST, pLeft, childFlags);

  /* purify the right side of the operator */
  if (soErr == aErrNone)
    soErr = sSOIExpressionProducer(pSAST, pRight, childFlags);

  /* make sure the types match (Left is mutable) */
  if (soErr == aErrNone) {
    if (sMatchTypes(pSAST, pLeft, pRight, &pNewLeft, &pNewRight,
                    aTrue, &soErr) == aTrue) {
      if (soErr == aErrNone) {
        if (pNewRight) {
          pLeft->pNext = pNewRight;
          pRight = pNewRight;
        } else if (pNewLeft) {
          pNewLeft->pNext = pRight;
          pLeft->pNext = NULL;
          pRelationalExpression->pChildren = pNewLeft;
          pLeft = pNewLeft;
        }
      }
    }
  }

  /* check for a signed / unsigned missmatch and constant
   * expressions */
  lType = (aSFlags)(pLeft->flags & fUnsigned);
  rType = (aSFlags)(pRight->flags & fUnsigned);
  op = pRelationalExpression->flags 
      		 & (fRelateEQ | fRelateLT | fRelateGT);
  if (lType != rType) {
    aBool bConstErr = aFalse;
    int val;
    if ((pRight->flags & fConstant) && lType) {
      val = (pRight->flags & fByte) 
      	     ? pRight->v.byteVal : pRight->v.shortVal;
      if ((val <= 0) && (op == fRelateLT))
        bConstErr = aTrue;
      if ((val < 0) && (op == (fRelateLT | fRelateEQ)))
        bConstErr = aTrue;
    } else if ((pLeft->flags & fConstant) && rType) {
      val = (pLeft->flags & fByte) 
      	     ? pLeft->v.byteVal : pLeft->v.shortVal;
      if ((val <= 0) && (op == fRelateGT))
        bConstErr = aTrue;
      if ((val < 0) && (op == (fRelateGT | fRelateEQ)))
        bConstErr = aTrue;
    } else
      soErr = pSAST->outputErrProc(pSAST, pRelationalExpression, 
    				   aTE_INCOMPAT_SIGN);
    if (bConstErr)
      soErr = pSAST->outputErrProc(pSAST, pRelationalExpression, 
       				   aTE_CONSTANT_RELATION);
  }

  /* precompute if constant */
  if ((soErr == aErrNone)
      && (pLeft->flags & fConstant)
      && (pRight->flags & fConstant)) {

    switch (op) {
    
    case (fRelateLT):
      if (pLeft->flags & fByte)
        pRelationalExpression->v.byteVal = 
	 (tBYTE)(((pLeft->v.byteVal < pRight->v.byteVal)) ? 1 : 0);
      else
        pRelationalExpression->v.byteVal = 
	 (tBYTE)(((pLeft->v.shortVal < pRight->v.shortVal)) ? 1 : 0);
      break;
      
    case (fRelateLT | fRelateEQ):
      if (pLeft->flags & fByte)
        pRelationalExpression->v.byteVal = 
	(tBYTE)(((pLeft->v.byteVal <= pRight->v.byteVal)) ? 1 : 0);
      else
        pRelationalExpression->v.byteVal = 
	(tBYTE)(((pLeft->v.shortVal <= pRight->v.shortVal)) ? 1 : 0);
      break;

    case (fRelateGT):
      if (pLeft->flags & fByte)
        pRelationalExpression->v.byteVal = 
	(tBYTE)(((pLeft->v.byteVal > pRight->v.byteVal)) ? 1 : 0);
      else
        pRelationalExpression->v.byteVal = 
	(tBYTE)(((pLeft->v.shortVal > pRight->v.shortVal)) ? 1 : 0);
      break;

    case (fRelateGT | fRelateEQ):
      if (pLeft->flags & fByte)
        pRelationalExpression->v.byteVal = 
	(tBYTE)(((pLeft->v.byteVal >= pRight->v.byteVal)) ? 1 : 0);
      else
        pRelationalExpression->v.byteVal = 
	(tBYTE)(((pLeft->v.shortVal >= pRight->v.shortVal)) ? 1 : 0);
      break;
    }    
    pRelationalExpression->flags |= fConstant;
  } /* if constant */

  /* set the type to byte */
  if (soErr == aErrNone)
    pRelationalExpression->flags |= fByte;

  return soErr;
  
} /* sSOIRelationalExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIEqualityExpression 
 *
 * used for == and != operations
 */

aErr sSOIEqualityExpression(aSAST* pSAST,
				   aASTNode* pEqualityExpression,
				   aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pLeft;
  aASTNode* pRight;
  aASTNode* pNewLeft = NULL;
  aASTNode* pNewRight = NULL;
  aSFlags childFlags = (aSFlags)((flags & ~fTypeMask) | fGetType);

  aAssert(pSAST);
  aAssert(pEqualityExpression);

  /* don't send down the type, we want to find it out */
  pEqualityExpression->flags |= (aSFlags)(flags & ~fTypeMask);

  pLeft = pEqualityExpression->pChildren;
  aAssert(pLeft);
  pRight = pLeft->pNext;
  aAssert(pRight);
  
  /* purify the left side of the operator */
  soErr = sSOIExpressionProducer(pSAST, pLeft, childFlags);

  /* purify the right side of the operator */
  if (soErr == aErrNone)
    soErr = sSOIExpressionProducer(pSAST, pRight, childFlags);

  /* make sure the types match (Left is mutable) */
  if (soErr == aErrNone) {
    if (sMatchTypes(pSAST, pLeft, pRight, &pNewLeft, &pNewRight,
                    aTrue, &soErr) == aTrue) {
      if (soErr == aErrNone) {
        if (pNewRight) {
          pLeft->pNext = pNewRight;
          pRight = pNewRight;
        } else if (pNewLeft) {
          pNewLeft->pNext = pRight;
          pLeft->pNext = NULL;
          pEqualityExpression->pChildren = pNewLeft;
          pLeft = pNewLeft;
        }
      }
    }    
  }    

  /* precompute if constant */
  if ((soErr == aErrNone)
      && (pLeft->flags & fConstant)
      && (pRight->flags & fConstant)) {
    if (pEqualityExpression->flags & fEquality) {
      if (pLeft->flags & fByte)
        pEqualityExpression->v.byteVal = (tBYTE)(((pLeft->v.byteVal == 
      					           pRight->v.byteVal)) ? 1 : 0);
      else
        pEqualityExpression->v.shortVal = (tSHORT)(((pLeft->v.shortVal == 
      					           pRight->v.shortVal)) ? 1 : 0);
    } else if (pEqualityExpression->flags & fNotEquality) {
      if (pLeft->flags & fByte)
        pEqualityExpression->v.byteVal = (tBYTE)(((pLeft->v.byteVal != 
      					           pRight->v.byteVal)) ? 1 : 0);
      else
        pEqualityExpression->v.shortVal = (tSHORT)(((pLeft->v.shortVal != 
      					           pRight->v.shortVal)) ? 1 : 0);
    }    
    pEqualityExpression->flags |= fConstant;
  }

  /* set the type to byte */
  if (soErr == aErrNone)
    pEqualityExpression->flags |= fByte;

  return soErr;
  
} /* sSOIEqualityExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOI2OpExpression 
 *
 * used for +, -, *, /, |, &, and ^ operations
 */

aErr sSOI2OpExpression(aSAST* pSAST,
		       aASTNode* p2OpExpression,
		       aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pLeft;
  aASTNode* pRight;
  aASTNode* pNewLeft = NULL;
  aASTNode* pNewRight = NULL;

  aAssert(pSAST);
  aAssert(p2OpExpression);

  p2OpExpression->flags |= flags;

  pLeft = p2OpExpression->pChildren;
  aAssert(pLeft);
  pRight = pLeft->pNext;
  aAssert(pRight);
  
  /* purify the left side of the operator */
  soErr = sSOIExpressionProducer(pSAST, pLeft, fGetType | fPushVal);

  /* purify the right side of the operator and find it's type */
  if (soErr == aErrNone)
    soErr = sSOIExpressionProducer(pSAST, pRight, 
    				   fGetType | fPushVal);

  /* make sure the types match (Left is mutable) */
  if (soErr == aErrNone) {
    if (sMatchTypes(pSAST, pLeft, pRight, &pNewLeft, &pNewRight,
                    aTrue, &soErr) == aTrue) {
      if (soErr == aErrNone) {
        if (pNewRight) {
          pLeft->pNext = pNewRight;
          pRight = pNewRight;
        } else if (pNewLeft) {
          pNewLeft->pNext = pRight;
          pLeft->pNext = NULL;
          p2OpExpression->pChildren = pNewLeft;
          pLeft = pNewLeft;
        }
      }
    }    
  }    

  /* next, make sure the resulting type is consistent with any 
   * that was requested */  
  if (soErr == aErrNone) {
    aSFlags resultType;
    aSFlags requestType = flags & fTypeMask;

    if (requestType) {
      resultType = pLeft->flags & fTypeMask;
      if (resultType != requestType)
        soErr = pSAST->outputErrProc(pSAST, pLeft, aTE_INCOMPAT_TYPE);
    }
  }

  /* precompute if constant */
  if ((soErr == aErrNone)
      && (pLeft->flags & fConstant)
      && (pRight->flags & fConstant)) {
    aBool bDivByZero = aFalse; 
    if (p2OpExpression->flags & f2OpOR) {
      if (pLeft->flags & fByte)
        p2OpExpression->v.byteVal = (tBYTE)(pLeft->v.byteVal | 
      					    pRight->v.byteVal);
      else
        p2OpExpression->v.shortVal = (tSHORT)(pLeft->v.shortVal | 
      					      pRight->v.shortVal);
    } else if (p2OpExpression->flags & f2OpXOR) {
      if (pLeft->flags & fByte)
        p2OpExpression->v.byteVal = (tBYTE)(pLeft->v.byteVal ^ 
      					    pRight->v.byteVal);
      else
        p2OpExpression->v.shortVal = (tSHORT)(pLeft->v.shortVal ^ 
      					      pRight->v.shortVal);
    } else if (p2OpExpression->flags & f2OpAND) {
      if (pLeft->flags & fByte)
        p2OpExpression->v.byteVal = (tBYTE)(pLeft->v.byteVal & 
      					    pRight->v.byteVal);
      else
        p2OpExpression->v.shortVal = (tSHORT)(pLeft->v.shortVal & 
      					      pRight->v.shortVal);
    } else if (p2OpExpression->flags & f2OpPLUS) {
      if (pLeft->flags & fByte)
        p2OpExpression->v.byteVal = (tBYTE)(pLeft->v.byteVal +
      					    pRight->v.byteVal);
      else
        p2OpExpression->v.shortVal = (tSHORT)(pLeft->v.shortVal + 
      					      pRight->v.shortVal);
    } else if (p2OpExpression->flags & f2OpMINUS) {
      if (pLeft->flags & fByte)
        p2OpExpression->v.byteVal = (tBYTE)(pLeft->v.byteVal -
      					    pRight->v.byteVal);
      else
        p2OpExpression->v.shortVal = (tSHORT)(pLeft->v.shortVal -
      					      pRight->v.shortVal);
    } else if (p2OpExpression->flags & f2OpMULT) {
      if (pLeft->flags & fByte)
        p2OpExpression->v.byteVal = (tBYTE)(pLeft->v.byteVal *
      					    pRight->v.byteVal);
      else
        p2OpExpression->v.shortVal = (tSHORT)(pLeft->v.shortVal *
      					      pRight->v.shortVal);
    } else if (p2OpExpression->flags & f2OpDIV) {
      if (pLeft->flags & fByte) {
        if (pRight->v.byteVal == 0)
          bDivByZero = aTrue;
        p2OpExpression->v.byteVal = (tBYTE)(pLeft->v.byteVal /
      					    pRight->v.byteVal);
      } else {
        if (pRight->v.shortVal == 0)
          bDivByZero = aTrue;
        p2OpExpression->v.shortVal = (tSHORT)(pLeft->v.shortVal /
      					      pRight->v.shortVal);
      }
    } else if (p2OpExpression->flags & f2OpMOD) {
      if (pLeft->flags & fByte) {
        if (pRight->v.byteVal == 0)
          bDivByZero = aTrue;
        p2OpExpression->v.byteVal = (tBYTE)(pLeft->v.byteVal %
      					    pRight->v.byteVal);
      } else {
        if (pRight->v.shortVal == 0)
          bDivByZero = aTrue;
        p2OpExpression->v.shortVal = (tSHORT)(pLeft->v.shortVal %
      					      pRight->v.shortVal);
      }
    }
    if (bDivByZero)
      soErr = pSAST->outputErrProc(pSAST, pRight, aTE_EXPR_DIV_ZERO);
    p2OpExpression->flags |= fConstant;
  }

  /* push up the type */
  if (soErr == aErrNone)
    p2OpExpression->flags |= (aSFlags)(pLeft->flags & fTypeMask);

  return soErr;
  
} /* sSOI2OpExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOILogicalOpExpression 
 *
 * used for both || and && operations
 */

aErr sSOILogicalOpExpression(aSAST* pSAST,
			      	     aASTNode* pLogicalOpExpression,
				     aSFlags flags)
{
  aErr soErr = aErrNone;
  aASTNode* pLeft;
  aASTNode* pRight;

  aAssert(pSAST);
  aAssert(pLogicalOpExpression);

  pLogicalOpExpression->flags |= (aSFlags)(flags | fByte);
  pLeft = pLogicalOpExpression->pChildren;
  aAssert(pLeft);
  pRight = pLeft->pNext;
  aAssert(pRight);
  
  /* purify the left side */
  soErr = sSOIExpressionProducer(pSAST, pLeft, flags);

  /* purify the right side */
  if (soErr == aErrNone)
    soErr = sSOIExpressionProducer(pSAST, pRight, flags);

  if ((soErr == aErrNone)
      && (pLeft->flags & fConstant)
      && (pRight->flags & fConstant)) {
    int v1, v2;

    /* show that this is a constant */
    pLogicalOpExpression->flags |= fConstant;

    /* compute the constant result */
    v1 = (pLeft->flags & fByte) ? pLeft->v.byteVal : pLeft->v.shortVal; 
    v2 = (pRight->flags & fByte) ? pRight->v.byteVal : pRight->v.shortVal; 
    switch (pLogicalOpExpression->eType) {
    case tLogicalORExpression:
      pLogicalOpExpression->v.byteVal = v1 || v2;
      break;
    case tLogicalANDExpression:
      pLogicalOpExpression->v.byteVal = v1 && v2;
      break;
    default:
      aAssert(0); /* should we get here ? */
      break;
    } /* switch */
  }

  return soErr;
  
} /* sSOILogicalOpExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIConditionalExpression 
 */

aErr sSOIConditionalExpression(aSAST* pSAST,
			       aASTNode* pIdentifier,
			       aSFlags flags)
{
  aErr soErr = aErrNone;
  
  return soErr;

} /* sSOIConditionalExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIStatementList 
 */

aErr sSOIStatementList(aSAST* pSAST,
			  aASTNode* pStatementList)
{
  aErr soErr = aErrNone;
  aASTNode* pStatement;
  
  aAssert(pStatementList);
  aAssert(pStatementList->eType == tStatementList);

  pStatement = pStatementList->pChildren;
  while ((pStatement != NULL) 
         && (soErr == aErrNone)) {
    soErr = sSOIStatement(pSAST, pStatement);
    pStatement = pStatement->pNext;
  }

  return soErr;

} /* sSOIStatementList */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIStatement
 */

aErr sSOIStatement(aSAST* pSAST,
		   aASTNode* pStatement)
{
  aErr soErr = aErrNone;
  aASTNode* pChild;

  aAssert(pStatement);
  aAssert(pStatement->eType == tStatement);

  pChild = pStatement->pChildren;
  aAssert(pChild);

  switch (pChild->eType) {
  case tExpressionStatement:
    soErr = sSOIExpressionStatement(pSAST, pChild);
    break;
  case tAsmStatement:
    soErr = sSOIAsmStatement(pSAST, pChild);
    break;
  case tJumpStatement:
    soErr = sSOIJumpStatement(pSAST, pChild);
    break;
  case tLabeledStatement:
    soErr = sSOILabeledStatement(pSAST, pChild);
    break;
  case tSelectionStatement:
    soErr = sSOISelectionStatement(pSAST, pChild);
    break;
  case tCompoundStatement:
    soErr = sSOICompoundStatement(pSAST, pChild);
    break;
  case tLoopStatement:
    soErr = sSOILoopStatement(pSAST, pChild);
    break;  
  default:
    aAssert(0); /* should we get here ? */
    break;
  } /* switch */

  /* cache this for optimization and return checking */
  if (soErr == aErrNone)
    pSAST->pLastStatement = pStatement;

  return soErr;

} /* sSOIStatement */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIAsmStatement
 */

aErr sSOIAsmStatement(aSAST* pSAST,
		      aASTNode* pAsmStatement)
{
  aErr soErr = aErrNone;
  aASTNode* pAsmList;

  aAssert(pAsmStatement);
  aAssert(pAsmStatement->eType == tAsmStatement);

  pAsmList = pAsmStatement->pChildren;

  if (pAsmList != NULL) {
    aASTNode* pChild = pAsmList->pChildren;
  
    while ((pChild != NULL)
           && (soErr == aErrNone)) {

      switch (pChild->eType) {

      case tOpcode:
        soErr = sSOIOpcode(pSAST, pChild);
        break;

      case tLabeledStatement:
        soErr = sSOILabeledStatement(pSAST, pChild);
        break;
  
      default:
        aAssert(0); /* should we get here ? */
        break;

      } /* switch */

      pChild = pChild->pNext;

    } /* while */
  }

  return soErr;

} /* sSOIAsmStatement */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIOpcode
 */

aErr sSOIOpcode(aSAST* pSAST,
		aASTNode* pOpcode)
{
  aErr soErr = aErrNone;
  tOpCode opCode;

  aAssert(pOpcode);
  aAssert(pOpcode->eType == tOpcode);

  opCode = (tOpCode)pOpcode->t.opcode.asmCode.code[0];

  switch (pSAST->opLengths[opCode]) {

  case 1:
    aAssert(pOpcode->pChildren);
    soErr = sSOIExpressionProducer(pSAST, pOpcode->pChildren, fByte);
    if (soErr == aErrNone) {
      if (!(pOpcode->pChildren->flags & fConstant))
        soErr = pSAST->outputErrProc(pSAST, pOpcode->pChildren, 
      				    aTE_CONST_EXPR_EXPECTED);
      else
        pOpcode->t.opcode.asmCode.code[1] = pOpcode->pChildren->v.byteVal;
    }
    break;

  case 2:
    aAssert(pOpcode->pChildren);
    if (aTEA_OpCodeTakesAddress(opCode) == aTrue) {
      soErr = sSOIIdentifier(pSAST, pOpcode->pChildren, 0);
    } else {
      soErr = sSOIExpressionProducer(pSAST, pOpcode->pChildren, 
      				     fShort | fGetType);
      if (soErr == aErrNone) {
        if (!(pOpcode->pChildren->flags & fConstant))
          soErr = pSAST->outputErrProc(pSAST, pOpcode->pChildren, 
      				 aTE_CONST_EXPR_EXPECTED);
        else
          aUtil_StoreShort(&(pOpcode->t.opcode.asmCode.code[1]), 
        		   (aSHORT)pOpcode->pChildren->v.shortVal);
      }
    }
    break;

  } /* switch */

  return soErr;

} /* sSOIOpcode */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIJumpStatement
 */

aErr sSOIJumpStatement(aSAST* pSAST,
		       aASTNode* pJumpStatement)
{
  aErr soErr = aErrNone;
  aASTNode* pExpression;
  aASTNode* pParent = NULL;

  aAssert(pJumpStatement);
  aAssert(pJumpStatement->eType == tJumpStatement);

  if (pJumpStatement->flags & fJumpReturn) {
    aSFlags flags = (aSFlags)(pSAST->pCurrentFunction->flags 
    			      & fTypeMask);
    pExpression = pJumpStatement->pChildren;

    /* get the return val's type */
    if (pExpression)
      soErr = sSOIExpression(pSAST, pExpression, 
      			     (aSFlags)(flags | fPushVal));
  } else if (pJumpStatement->flags & fJumpBreak) {
    pParent = pJumpStatement->pParent;
    while (pParent 
           && !(pParent->eType == tFunctionDefinition)
           && !(pParent->flags & fBreakable))
      pParent = pParent->pParent;
    
    if (!pParent || !(pParent->flags & fBreakable)) {
      soErr = pSAST->outputErrProc(pSAST, pJumpStatement, 
      				  aTE_BREAK_OUTSIDE_BREAKABLE);
     } else {
      pJumpStatement->pRef = pParent;
    }
    
  /* continue statements need to reside within a loop construct */
  } else if (pJumpStatement->flags & fJumpContinue) {
    pParent = pJumpStatement->pParent;
    while (pParent 
           && !(pParent->eType == tFunctionDefinition)
           && !(pParent->eType == tLoopStatement))
      pParent = pParent->pParent;

    if (!pParent || !(pParent->eType == tLoopStatement)) {
      soErr = pSAST->outputErrProc(pSAST, pJumpStatement, 
      				   aTE_CONTINUE_NOT_IN_LOOP);
    } else {
      /* point to the loop statement with the ref */
      pJumpStatement->pRef = pParent;
    }
  }

  /* goto is not handled here because it may be a forward address
   * reference, see sSOIAddressReferences. */

  return soErr;

} /* sSOIJumpStatement */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIExpression
 */

aErr sSOIExpression(aSAST* pSAST,
		    aASTNode* pExpression,
		    aSFlags callerFlags)
{
  aErr soErr = aErrNone;
  aASTNode* pChild;
  aSFlags childFlags;
  
  aAssert(pExpression);
  aAssert(pExpression->eType == tExpression);

  /* if our caller knows what type we should be, ask our
   * children what type they are and upcast or error 
   * appropriately, otherwise, just get the type */
  if (callerFlags & fGetType) {
    childFlags = callerFlags;
  } else {
    childFlags = callerFlags | fGetType;
    childFlags &= ~fTypeMask;
  }

  pExpression->flags |= callerFlags;

  pChild = pExpression->pChildren;

  /* walk through each sub-expression and purify it */
  while ((pChild != NULL)
         && (soErr == aErrNone)) {
 
    /* get the type of each child */
    soErr = sSOIExpressionProducer(pSAST, pChild, childFlags);
    
    /* if the caller knew what type we should be, verify (and fix) it */
    if (!(callerFlags & fGetType)) {
      aSFlags parentType = pExpression->flags & fBaseTypeMask;
      aSFlags childType = pChild->flags & (fBaseTypeMask & ~fAddress);
      if ((parentType != 0)
          && (parentType != childType)) {
        if (sTypeCanPromote(parentType, childType) == aTrue) {
          aASTNode* pCast;
          aASTNode* pDeclSpec;
	  soErr = sBuildAutoCast(pSAST, fShort, &pCast, &pDeclSpec);
          
          /* splice it in */
          if (soErr == aErrNone) {
            /* stick the old expression under the cast */
            pDeclSpec->pNext = pChild;
            pExpression->pChildren = pCast;
            /* changeling.... ee ee ee ee */
            pChild = pCast;
            soErr = sSOIExpressionProducer(pSAST, pCast,
    				  (aSFlags)(fGetType | fPushVal));
          }
        } else {
          soErr = pSAST->outputErrProc(pSAST, pChild, aTE_INCOMPAT_TYPE);
        }
      }
    }
    if ((soErr == aErrNone)
        && (pChild->pNext == NULL))
      soErr = aAST_PushUpFlags(pExpression, pChild);
    pChild = pChild->pNext;
  }

  return soErr;

} /* sSOIExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIExpressionStatement
 */

aErr sSOIExpressionStatement(aSAST* pSAST,
		       	     aASTNode* pExpressionStatement)
{
  aErr soErr = aErrNone;
  aASTNode* pExpression;

  aAssert(pExpressionStatement);
  aAssert(pExpressionStatement->eType == tExpressionStatement);

  pExpression = pExpressionStatement->pChildren;

  while ((pExpression != NULL)
         && (soErr == aErrNone)) {
    soErr = sSOIExpression(pSAST, pExpression, 0);
    pExpression = pExpression->pNext;
  }

  return soErr;

} /* sSOIExpressionStatement */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOILabeledStatement
 */

aErr sSOILabeledStatement(aSAST* pSAST,
			  aASTNode* pLabeledStatement)
{
  aErr soErr = aErrNone;

  aAssert(pLabeledStatement);
  aAssert(pLabeledStatement->eType == tLabeledStatement);

  if (pLabeledStatement->flags & (fLabelCase | fLabelDefault)) {
    aASTNode* pExpression = pLabeledStatement->pChildren;

    /* check out the case label's constant expression */
    if (pLabeledStatement->flags & fLabelCase) {
      aAssert(pExpression);
      soErr = sSOIExpressionProducer(pSAST, pExpression, 
    				     fGetType | fPushVal);
      if ((soErr == aErrNone)
          && !(pExpression->flags & fConstant))
        soErr = pSAST->outputErrProc(pSAST, pExpression, 
      				    aTE_CONST_EXPR_EXPECTED);
    }

    /* need to point to the select switch with the ref */
    if ((soErr == aErrNone)) {
      aASTNode* pParent = pLabeledStatement->pParent;
      while (pParent 
             && (pParent->eType != tSelectionStatement)
             && !(pParent->flags & fSelectSwitch))
        pParent = pParent->pParent;
      
      /* establish some links to ease code generation */
      if (pParent) {

        if (pLabeledStatement->flags & fLabelCase) {

          aSFlags switchType;
          aSFlags caseType;
          aASTNode* pSwitchExpression = pParent->pChildren;
          switchType = (pSwitchExpression->flags & fTypeMask);
          caseType = (pExpression->flags & fTypeMask);

          /* first, check type compatibility */
          if (switchType != caseType) { 
            if (sTypeCanPromote(switchType, caseType) == aTrue) {

              aASTNode* pCast;
              aASTNode* pDeclSpec;
	      soErr = sBuildAutoCast(pSAST, fShort, &pCast, &pDeclSpec);

              /* splice it in */
              if (soErr == aErrNone) {
                /* stick the old expression under the cast */
                pDeclSpec->pNext = pExpression->pChildren;
                pExpression->pChildren = pCast;
                /* changeling.... ee ee ee ee, recompute the expression
                 * to allow optimization of the cast */
                 /* MRW -- pCast or pExpression with type cleared ??? */
                 pExpression->flags &= (~fTypeMask);
                soErr = sSOIExpressionProducer(pSAST, pExpression,
                                               (aSFlags)(fGetType | fPushVal));
              }

            } else {
              soErr = pSAST->outputErrProc(pSAST, pExpression, 
      				        aTE_INCOMPAT_TYPE);
            }
          }
        }

        /* build a link chain starting at the switch using 
         * ref as a cached pointer to the previous labeled 
         * statement */
      	if (pParent->pRef)
          pParent->pRef->t.switchChain.pNext = pLabeledStatement;
        else
          pParent->t.switchChain.pNext = pLabeledStatement;
        pParent->pRef = pLabeledStatement;

        /* point back to the switch */
        pLabeledStatement->pRef = pParent;
      }  else
        soErr = pSAST->outputErrProc(pSAST, pLabeledStatement, 
      				    aTE_CASE_OUTSIDE_SWITCH);
    }
  }

  return soErr;

} /* sSOILabeledStatement */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOISelectionStatement
 */

aErr sSOISelectionStatement(aSAST* pSAST,
			    aASTNode* pSelectionStatement)
{
  aErr soErr = aErrNone;
  aASTNode* pExpression;
  aASTNode* pStatement;

  aAssert(pSelectionStatement);
  aAssert(pSelectionStatement->eType == tSelectionStatement);

  /* advertise that switches are breakable (before statement purify */
  if (pSelectionStatement->flags & fSelectSwitch)
    pSelectionStatement->flags |= fBreakable;

  pExpression = pSelectionStatement->pChildren;
  aAssert(pExpression);

  /* handle the expression code */
  soErr = sSOIExpression(pSAST, pExpression, fGetType | fPushVal);

  /* handle the actual selection code */
  pStatement = pExpression->pNext; 
 
  if ((soErr == aErrNone) && pStatement) {
    soErr = sSOIStatement(pSAST, pStatement);
    if ((soErr == aErrNone)
        && (pStatement->pNext))
      soErr = sSOIStatement(pSAST, pStatement->pNext);
  }
  
  /* now check for duplicate cases in switch statements */
  if ((soErr == aErrNone)
      && (pSelectionStatement->flags & fSelectSwitch)) {
    aASTNode* pDefault = NULL;
    aASTNode* pChain = pSelectionStatement->t.switchChain.pNext;

    while (pChain) {
      if (pChain->flags & fLabelDefault) {
        if (pDefault)
          soErr = pSAST->outputErrProc(pSAST, pChain, 
      				      aTE_DUPLICATE_CASE);
        if (soErr == aErrNone)
          pDefault = pChain;
      } else {
        aASTNode* pTemp = pChain->t.switchChain.pNext;
        while (pTemp) {
          if ((pTemp->flags & fByte)
              && (pTemp->v.byteVal == pChain->v.byteVal))
            soErr = pSAST->outputErrProc(pSAST, pTemp, 
      				        aTE_DUPLICATE_CASE);
          else if ((pTemp->flags & fShort)
                   && (pTemp->v.shortVal == pChain->v.shortVal))
            soErr = pSAST->outputErrProc(pSAST, pTemp, 
      				        aTE_DUPLICATE_CASE);
          pTemp = pTemp->t.switchChain.pNext;
        }
      }
      pChain = pChain->t.switchChain.pNext;
    }
    /* need to link the default at the end of the chain */
    if (pDefault && pDefault->t.switchChain.pNext) {
      aASTNode* pPrev = NULL;
      /* first, remove the default from the list */
      pChain = pSelectionStatement->t.switchChain.pNext;
      while (pChain) {
        if (pChain == pDefault) {
          if (pPrev == NULL)
            pSelectionStatement->t.switchChain.pNext = 
              pChain->t.switchChain.pNext;
          else 
            pPrev->t.switchChain.pNext = 
              pChain->t.switchChain.pNext;
          break;
        }
        pChain = pChain->t.switchChain.pNext;
      }
      /* then insert it at the end */
      pPrev = NULL;
      pChain = pSelectionStatement->t.switchChain.pNext;
      while (pChain) {
        pPrev = pChain;
        pChain = pChain->t.switchChain.pNext;
      }
      aAssert(pPrev);
      aAssert(pChain == NULL);
      pPrev->t.switchChain.pNext = pDefault;
      pDefault->t.switchChain.pNext = 0;
    }
  }

  return soErr;

} /* sSOISelectionStatement */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOILoopStatement
 */

aErr sSOILoopStatement(aSAST* pSAST,
		       aASTNode* pLoopStatement)
{
  aErr soErr = aErrNone;
  aASTNode* pChild;

  aAssert(pLoopStatement);
  aAssert(pLoopStatement->eType == tLoopStatement);

  /* advertise that loops are breakable */
  pLoopStatement->flags |= fBreakable;

  pChild = pLoopStatement->pChildren;

  while (pChild && (soErr == aErrNone)) {
  
    switch (pChild->eType) {
 
    case tExpression:
      /* we only want the loop condition left on the stack, the 
       * others just get evaluated */
      if (pChild->flags & fLoopCondition)
        soErr = sSOIExpression(pSAST, pChild, fGetType | fPushVal);
      else
        soErr = sSOIExpression(pSAST, pChild, fGetType);
      break;

    case tStatement:
      soErr = sSOIStatement(pSAST, pChild);
      break;
  
    default:
      aAssert(0); /* should we get here ? */
      break;

    } /* switch */
    
    pChild = pChild->pNext;

  } /* while */

  return soErr;

} /* sSOILoopStatement */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSOIExpressionProducer
 */
 
aErr sSOIExpressionProducer(
  aSAST* pSAST,
  aASTNode* pNode,
  aSFlags flags)
{
  aErr soErr = aErrNone;

  switch (pNode->eType) {
  case tExpression:
    soErr = sSOIExpression(pSAST, pNode, flags);
    break;
  case tAssignmentExpression:
    soErr = sSOIAssignmentExpression(pSAST, pNode, flags);
    break;
  case tConditionalExpression:
    soErr = sSOIConditionalExpression(pSAST, pNode, flags);
    break;
  case tLogicalORExpression:
  case tLogicalANDExpression:
    soErr = sSOILogicalOpExpression(pSAST, pNode, flags);
    break;
  case tInclusiveORExpression:
  case tExclusiveORExpression:
  case tANDExpression:
  case tAdditiveExpression:
  case tMultiplicativeExpression:
    soErr = sSOI2OpExpression(pSAST, pNode, flags);
    break;
  case tEqualityExpression:
    soErr = sSOIEqualityExpression(pSAST, pNode, flags);
    break;
  case tRelationalExpression:
    soErr = sSOIRelationalExpression(pSAST, pNode, flags);
    break;
  case tShiftExpression:
    soErr = sSOIShiftExpression(pSAST, pNode, flags);
    break;
  case tCastExpression:
    soErr = sSOICastExpression(pSAST, pNode, flags);
    break;
  case tUnaryExpression:
    soErr = sSOIUnaryExpression(pSAST, pNode, flags);
    break;
  case tUnaryOperator:
    soErr = sSOIUnaryOperator(pSAST, pNode, flags);
    break;
  case tPostFixExpression:
    soErr = sSOIPostFixExpression(pSAST, pNode, flags);
    break;
  case tPrimaryExpression:
    soErr = sSOIPrimaryExpression(pSAST, pNode, flags);
    break;  
  default:
    aAssert(0); /* should we get here ? */
    break;
  } /* switch */
  
  return soErr;

} /* sSOIExpressionProducer */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepOptInternal_Create
 */

aErr aSteepOptInternal_Create(aSteepOpt** ppSteepOpt)
{
  aErr soErr = aErrNone;
  aSteepOpt* pSteepOpt = NULL;

  if (ppSteepOpt == NULL)
    soErr = aErrParam;

  if (soErr == aErrNone) {
    pSteepOpt = (aSteepOpt*)aMemAlloc(sizeof(aSteepOpt));
    if (pSteepOpt == NULL) {
      soErr = aErrParam;
    } else {
      aBZero(pSteepOpt, sizeof(aSteepOpt));
      pSteepOpt->check = aSTEEPOPTCHECK;
      if (soErr != aErrNone)
        aSteepOptInternal_Destroy(pSteepOpt);
    }
  }
  
  if (soErr == aErrNone)
    *ppSteepOpt = pSteepOpt;
  
  return soErr;

} /* aSteepOptInternal_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepOptInternal_Optimize
 */

aErr aSteepOptInternal_Optimize(aSteepOpt* pSteepOpt,
				aSAST* pSAST)
{
  aErr soErr = aErrNone;
  aASTNode* pTranslationUnit = pSAST->pTree;

  aAssert(pTranslationUnit);
  aAssert(pTranslationUnit->eType == tTranslationUnit);

  /* build a root symbol table for the globals */
  if (soErr == aErrNone) {
    aAssert(pTranslationUnit->symbolTable == NULL);
    aSymbolTable_Create(pSAST->ioRef, 
    			&pTranslationUnit->symbolTable, &soErr);
  }

  /* walk the entire tree and insert all variables, labels,
   * and function names into the symbol table */
  if (soErr == aErrNone)
    soErr = sSOICollectSymbols(pSAST, pTranslationUnit);

  /* now, do a pass to resolve all address references such
   * as subroutine calls and goto statements.  These may be
   * forward references.  For routine calls, set the return
   * type for future casting and type checking */
  if (soErr == aErrNone)
    soErr = sSOIAddressReferences(pSAST, pTranslationUnit);

  /* now, fold constants, promote types, and resolve 
   * routine parameters and types */
  if (soErr == aErrNone)
    soErr = sSOITranslationUnit(pSAST, pTranslationUnit);

  /* finally, tell the backend where the main routine is 
   * and clean up the symbol table */
  if ((soErr == aErrNone)
       && (pTranslationUnit->symbolTable != NULL)) {
    aErr symErr;
    aSymbolTableRef globals;
    aASTNode* pTemp;
    void* vpSymData;

    /* first, find the main routine */
    globals = pSAST->pTree->symbolTable;
    aAssert(globals);
    aSymbolTable_Find(pSAST->ioRef, globals, aTR_MAIN, 
    		      &vpSymData, &symErr);
    if (symErr == aErrNone) {
      pTemp = ((aASTNodeSym*)vpSymData)->pNode;
      aAssert(pTemp);
      while (pTemp && (pTemp->eType != tFunctionDefinition))
        pTemp = pTemp->pParent;
      aAssert(pTemp);
      /* mark the routine as being used */
      pSAST->pMainRoutine = pTemp;
    } else {
      soErr = pSAST->outputErrProc(pSAST, NULL, aTE_MAIN_MISSING);
    }

    /* clean up all the symbol tables */
    if (soErr == aErrNone)
      soErr = sSOICleanUp(pSAST, pTranslationUnit);

    /* mark all the routines spreading out from main as used */
    if (soErr == aErrNone)
      soErr = sSOIMarkUsed(pSAST, pSAST->pMainRoutine);
  }
  
  return soErr;

} /* aSteepOptInternal_Optimize */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepOptInternal_Destroy
 */

aErr aSteepOptInternal_Destroy(aSteepOpt* pSteepOpt)
{
  aErr soErr = aErrNone;

  aVALIDSTEEPOPT(pSteepOpt);

  aAssert(soErr == aErrNone);

  if (soErr == aErrNone) {
    pSteepOpt->check = 0;
    aMemFree((aMemPtr)pSteepOpt);
  }
  
  return soErr;

} /* aSteepOptInternal_Destroy */
