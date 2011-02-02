/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSteepGenInternal.c					   */
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
#include "aSteepText.h"
#include "aSteepGenInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */


static aErr sBE_GetCallSizes(aASTNode* pFunction,
			     tBYTE* pnRetValSize,
			     tBYTE* pnParamListSize);
static aErr sBE_GenFunctionDefinition(aSAST* pSAST, 
				      aASTNode* pFunction);
static aErr sBE_GenDeclarationList(aSAST* pSAST, 
				   aASTNode* pDeclarationList);
static aErr sBE_GenDeclaration(aSAST* pSAST, 
			       aASTNode* pDeclaration);
static aErr sBE_GenInitDeclarator(aSAST* pSAST, 
				  aASTNode* pInitDeclarator);
static aErr sBE_GenInitializer(aSAST* pSAST,
			       aASTNode* pInitializer);
static aErr sBE_GenUnaryOperator(aSAST* pSAST,
			         aASTNode* pUnaryOperator);
static aErr sBE_GenAssignmentExpression(aSAST* pSAST,
					aASTNode* pAssignmentExpression);
static aErr sBE_GenUnaryExpression(aSAST* pSAST,
				   aASTNode* pUnaryExpression);
static aErr sBE_GenPostFixExpression(aSAST* pSAST,
				     aASTNode* pPostFixExpression);
static aErr sBE_GenPrimaryExpression(aSAST* pSAST,
				     aASTNode* pPrimaryExpression);
static aErr sBE_GenConstant(aSAST* pSAST,
			    aASTNode* pConstant);
static aErr sBE_GenString(aSAST* pSAST,
			  aASTNode* pString);
static aErr sBE_GenDeclarator(aSAST* pSAST,
    			      aASTNode* pTemp);
static aErr sBE_GenIdentifier(aSAST* pSAST,
			      aASTNode* pIdentifier);
static aErr sBE_GenParameterList(aSAST* pSAST,
				 aASTNode* pParameterList);
static aErr sBE_GenStatementList(aSAST* pSAST,
				 aASTNode* pStatementList);
static aErr sBE_GenStatement(aSAST* pSAST,
			     aASTNode* pStatement);
static aErr sBE_GenAsmStatement(aSAST* pSAST,
			        aASTNode* pAsmStatement);
static aErr sBE_GenOpcode(aSAST* pSAST,
			  aASTNode* pOpcode);
static aErr sBE_GenJumpStatement(aSAST* pSAST,
				 aASTNode* pJumpStatement);
static aErr sBE_GenExpression(aSAST* pSAST,
			      aASTNode* pExpression);
static aErr sBE_GenExpressionStatement(aSAST* pSAST,
				       aASTNode* pExpressionStatement);
static aErr sBE_GenLabeledStatement(aSAST* pSAST,
				    aASTNode* pLabeledStatement);
static aErr sBE_GenSelectionStatement(aSAST* pSAST,
				      aASTNode* pSelectionStatement);
static aErr sBE_GenLoopStatement(aSAST* pSAST,
				 aASTNode* pLoopStatement);
static aErr sBE_GenCompoundStatement(aSAST* pSAST,
				     aASTNode* pCompoundStatement);
static aErr sBE_GenExpressionProducer(aSAST* pSAST,
				      aASTNode* pNode);
static aErr sBE_GenLogicalORExpression(aSAST* pSAST,
				       aASTNode* pNode);
static aErr sBE_GenLogicalANDExpression(aSAST* pSAST,
				        aASTNode* pNode);
static aErr sBE_Gen2OpExpression(aSAST* pSAST,
				       aASTNode* pNode);
static aErr sBE_GenEqualityExpression(aSAST* pSAST,
				      aASTNode* pEqualityExpression);
static aErr sBE_GenRelationalExpression(aSAST* pSAST,
				        aASTNode* pRelationalExpression);
static aErr sBE_GenShiftExpression(aSAST* pSAST,
				   aASTNode* pShiftExpression);
static aErr sBE_GenCastExpression(aSAST* pSAST,
				  aASTNode* pCastExpression);

static tBYTE sTypeSizeFromFlags(aASTNode* pNode);

static aASTNode* sGetIdentifier(aASTNode* pLValue);

static aErr sPopToIdentifierRef(aSAST* pSAST,
				aASTNode* pIdentifier);
static aErr sPushFromIdentifierRef(aSAST* pSAST,
				  aASTNode* pIdentifier);

static aErr sPopExpression(aSAST* pSAST,
			   aASTNode* pExpression);

static aErr sOutputRoutine(aSAST* pSAST, 
			   aASTNode* pRoutine);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTypeSizeFromFlags
 */

tBYTE sTypeSizeFromFlags(aASTNode* pNode)
{
  if (pNode->flags & fShort)
    return sizeof(tSHORT);
  if (pNode->flags & fByte)
    return sizeof(tBYTE);
  if (pNode->flags & fAddress)
    return sizeof(tADDRESS);
  if (pNode->flags & fString) {
    if ((pNode->eType == tPrimaryExpression)
        && pNode->pChildren
        && pNode->pChildren->pToken
        && (pNode->pChildren->eType == tString)) {
      return (tBYTE)(pNode->pChildren->t.string.length + 1);
    }
  }
  
  return 0;

} /* sTypeSizeFromFlags */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sGetIdentifier
 */

aASTNode* sGetIdentifier(aASTNode* pLValue)
{
  aASTNode* pTemp = pLValue;
  while (pTemp && (pTemp->eType != tIdentifier)) 
    pTemp = pTemp->pChildren;

  return pTemp;

} /* sGetIdentifier */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPopToIdentifierRef
 *
 * takes the value on the stack (based on the identifier) and stores
 * it into the identifier's referenced location.  Basically this is
 * an assignment operation used by =, +=, ++, etc.
 */

aErr sPopToIdentifierRef(aSAST* pSAST,
		         aASTNode* pIdentifier)
{
  aErr sgErr = aErrNone;
  
  aAssert(pIdentifier);
  aAssert(pIdentifier->eType == tIdentifier);

  if (pSAST->bJustCompute) {

    if (pIdentifier->pRef->flags & fGlobal)
      /* space for POPSXA */
      pSAST->nCodeBytes += 3;
    else
      /* space for the POPSX */
      pSAST->nCodeBytes += 2;
    
  } else {
    
    /* account for the pop of the assignment */
    pSAST->nStackOffset -= sTypeSizeFromFlags(pIdentifier);

    /* globals use an absolute stack reference which is two bytes */
    if (pIdentifier->pRef->flags & fGlobal) {

      /* generate the code for popsXa */
      if (pIdentifier->flags & fShort) {
        pSAST->code[pSAST->nCodeBytes++] = op_POPSSA;
      } else if (pIdentifier->flags & fByte) {
        pSAST->code[pSAST->nCodeBytes++] = op_POPBSA;
      }

      /* the reference is absolute */
      aTEA_StoreStack(&pSAST->code[pSAST->nCodeBytes], 
      		      pIdentifier->pRef->t.identifier.stackPos);
      pSAST->nCodeBytes += sizeof(tSTACK);

    } else {

      /* generate the code for popsX */
      if (pIdentifier->flags & fShort) {
        pSAST->code[pSAST->nCodeBytes++] = op_POPSS;
      } else if (pIdentifier->flags & fByte) {
        pSAST->code[pSAST->nCodeBytes++] = op_POPBS;
      }
      
      /* the reference is the total stack minus 
       * the identifier's location on it
       * minus the global stack size if main and local */
      pSAST->code[pSAST->nCodeBytes++] = 
    	(tBYTE)(pSAST->pCurrentFunction->t.function.stackSize 
    		- pIdentifier->pRef->t.identifier.stackPos 
    		+ pSAST->nStackOffset
    		);
    }

    /* finally, show that the variable being assigned has now
     * been initialized */
    pIdentifier->pRef->flags |= fIdentInited;
  }

  return sgErr;

} /* sPopToIdentifierRef */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPushFromIdentifierRef
 */

aErr sPushFromIdentifierRef(aSAST* pSAST,
			    aASTNode* pIdentifier)
{
  aErr sgErr = aErrNone;
  
  aAssert(pIdentifier);
  aAssert(pIdentifier->flags & fPushVal);
  aAssert(pIdentifier->pRef);

  if (pSAST->bJustCompute) {

      if (pIdentifier->pRef->flags & fGlobal)
        pSAST->nCodeBytes += 3;
      else
        pSAST->nCodeBytes += 2;

  } else {

      /* create an error if the variable being referenced
       * has not been initialized */
      if (!(pIdentifier->pRef->flags & fIdentInited)) {
        pSAST->outputErrProc(pSAST, pIdentifier, 
        		     aTE_VARIABLE_NOT_INITED);
      }

      if (pIdentifier->pRef->flags & fGlobal) {
        /* generate the code for pushsX */
        if (pIdentifier->flags & fShort)
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHSSA;
        else if (pIdentifier->flags & fByte)
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHSBA;
        /* the reference is absolute */
        aTEA_StoreStack(&pSAST->code[pSAST->nCodeBytes], 
        		pIdentifier->pRef->t.identifier.stackPos);
        pSAST->nCodeBytes += sizeof(tSTACK);
      } else {

        /* generate the code for pushsX */
        if (pIdentifier->flags & fShort)
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHSS;
        else if (pIdentifier->flags & fByte)
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHSB;
          
        /* the reference is the total stack minus 
         * the identifier's location on it
         * minus the global stack size if main and local */
        pSAST->code[pSAST->nCodeBytes++] = 
	    (tBYTE)(pSAST->pCurrentFunction->t.function.stackSize 
    		- pIdentifier->pRef->t.identifier.stackPos
    	 	+ pSAST->nStackOffset 
    		);
      }

      /* now account for this on the stack */
      pSAST->nStackOffset += sTypeSizeFromFlags(pIdentifier);
  }

  return sgErr;

} /* sPushFromIdentifierRef */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPopExpression
 */

aErr sPopExpression(aSAST* pSAST,
		    aASTNode* pExpression)
{
  aErr sgErr = aErrNone;
  
  aAssert(pSAST);
  aAssert(pExpression);
/*  aAssert(pExpression->flags & fPushVal); */

  if (pSAST->bJustCompute) {
    if (pExpression->flags & (fByte | fShort))
      pSAST->nCodeBytes++;
  } else {
    if (pExpression->flags & fByte) {
      pSAST->code[pSAST->nCodeBytes++] = op_POPB;
      pSAST->nStackOffset -= 1;
    } else if (pExpression->flags & fShort) {
      pSAST->code[pSAST->nCodeBytes++] = op_POPS;
      pSAST->nStackOffset -= 2;
    }
  }

  return sgErr;

} /* sPopExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sOutputRoutine
 */

aErr sOutputRoutine(aSAST* pSAST,
		    aASTNode* pRoutine)
{
  aErr sgErr = aErrNone;
  tADDRESS nCodeBytes = pRoutine->nCodePosition;

  aAssert(pSAST->code);

  /* set up the SAST block for code generation */
  pSAST->nCodeBytes = 0;
  pSAST->pCurrentFunction = pRoutine;
  pSAST->bJustCompute = aFalse;
      
  sgErr = sBE_GenFunctionDefinition(pSAST, pRoutine);

  if (sgErr == aErrNone)
    aStream_Write(aStreamLibRef(pSAST->outputStream),
  		    		pSAST->outputStream,
  		    		pSAST->code,
  		    		nCodeBytes,
  		    		&sgErr);
   
   aAssert(pSAST->nCodeBytes == nCodeBytes);
  
   return sgErr;

} /* sOutputRoutine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GetCallSizes
 */

aErr sBE_GetCallSizes(aASTNode* pFunction,
		      tBYTE* pnRetValSize,
		      tBYTE* pnParamListSize)
{
  aErr sgErr = aErrNone;
  aASTNode* pTemp;
  
  /* the return value size */
  if (pnRetValSize != NULL) {
    pTemp = pFunction->pChildren;
    aAssert(pTemp->eType == tDeclarationSpecifier);
    *pnRetValSize = sTypeSizeFromFlags(pTemp);
  }
  
  /* the parameter list size */
  if (pnParamListSize != NULL) {
    *pnParamListSize = 0;
    pTemp = pTemp->pNext;
    aAssert(pTemp->eType == tDeclarator);
    pTemp = pTemp->pChildren;
    aAssert(pTemp->eType == tIdentifier);
    pTemp = pTemp->pNext;
    aAssert(pTemp->eType == tParameterList);
    pTemp = pTemp->pChildren;
    while (pTemp != NULL) {
      aASTNode* pDeclSpec = pTemp->pChildren;
      aAssert(pTemp->eType == tParameterDeclaration);
      aAssert(pDeclSpec->eType == tDeclarationSpecifier);
      *pnParamListSize += sTypeSizeFromFlags(pDeclSpec);
      pTemp = pTemp->pNext;
    }
  }

  return sgErr;

} /* sBE_GetCallSizes */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenFunctionDefinition
 */

aErr sBE_GenFunctionDefinition(aSAST* pSAST, 
			       aASTNode* pFunction)
{
  aErr sgErr = aErrNone;
  aASTNode* pTemp;
  unsigned char nReturnBytes;

  aAssert(pSAST);
  aAssert(pFunction);
  aAssert(pFunction->eType == tFunctionDefinition);

  /* first, calculate the size of the routine stack 
   * this always looks like:
   *   return value (possibly none)
   *   parameters (possibly none)
   *   globals (when we are the main routine)
   *   return address (tADDRESS)
   *   local stack variables
   */
  if (pSAST->bJustCompute == aTrue) {

    /* this will accumulate the total number of bytes
     * in the routine */
    pSAST->nCodeBytes = 0;

    /* start with an empty stack */
    pFunction->t.function.stackSize = 0;

    /* the return value size */
    pTemp = pFunction->pChildren;
    aAssert(pTemp->eType == tDeclarationSpecifier);
    nReturnBytes = (unsigned char)sTypeSizeFromFlags(pTemp);

    /* set the current function for subordinate routine calls */
    pSAST->pCurrentFunction = pFunction;

    /* establish the routine's stack size accumulator */
    pFunction->t.function.stackSize += nReturnBytes;
 
    /* the size of the parameters get added to the 
     * function's stackSize */
    pTemp = pTemp->pNext;
    aAssert(pTemp->eType == tDeclarator);
    sgErr = sBE_GenDeclarator(pSAST, pTemp);
/*
    pFunction->t.function.stackSize += 
    	(unsigned char)sTypeSizeFromFlags(pTemp);
 */

    /* if this is the main routine, globals are part of the 
     * routine stack size */
    if (pFunction == pSAST->pMainRoutine)
      pFunction->t.function.stackSize += 
      	(unsigned char)pSAST->nGlobalStackSize;

    /* Add the return address pushed by the CALL opcode
     * on entry into the routine. */
    pFunction->t.function.stackSize += sizeof(tADDRESS);

    /* store the completed stack size into the function's data */
    pFunction->t.function.callSize = pFunction->t.function.stackSize;

    /* now, handle the compound statement */
    pTemp = pTemp->pNext;
    aAssert(pTemp->eType == tCompoundStatement);
    sgErr = sBE_GenCompoundStatement(pSAST, pTemp);

    /* keep track of the routine code size */
    pFunction->nCodePosition = pSAST->nCodeBytes;

  /* otherwise, generate the code */     
  } else {

    /* make sure we have code preparation area to write to */
    aAssert(pSAST->code);
 
    /* the return value size */
    pTemp = pFunction->pChildren;
    aAssert(pTemp->eType == tDeclarationSpecifier);
    pTemp = pTemp->pNext;
    aAssert(pTemp->eType == tDeclarator);

    pSAST->nStackOffset = 0;

    if (sgErr == aErrNone) {
      pTemp = pTemp->pNext;
      aAssert(pTemp->eType == tCompoundStatement);
      sgErr = sBE_GenCompoundStatement(pSAST, pTemp);
    }

    /* should have cleaned up the stack */
    aAssert(pSAST->nStackOffset == 0);
  }

  return sgErr;

} /* sBE_GenFunctionDefinition */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenDeclarationList
 */

aErr sBE_GenDeclarationList(aSAST* pSAST, 
			    aASTNode* pDeclarationList)
{
  aErr sgErr = aErrNone;
  aASTNode* pDeclaration;
  tSTACK size = 0;

  aAssert(pDeclarationList);
  aAssert(pDeclarationList->eType == tDeclarationList);

  /* first, compute the stack size */
  if (pSAST->bJustCompute == aTrue) {

    /* cache the current stack size before the declarations */
    size = pSAST->pCurrentFunction->t.function.stackSize;

  } 

  /* store the position of this node in the output code */
  pDeclarationList->nCodePosition = pSAST->nCodeBytes;

  /* step through each declaration */
  pDeclaration = pDeclarationList->pChildren;    
  while ((pDeclaration != NULL)
         && (sgErr == aErrNone)) {
    sgErr = sBE_GenDeclaration(pSAST, pDeclaration);
    pDeclaration = pDeclaration->pNext;
  } /* while */

  /* store the size of the declaration variables for cleanup 
   * on exit */
  if (pSAST->bJustCompute == aTrue) {
    pDeclarationList->t.declarationList.stackSize = 
    	(unsigned char)(pSAST->pCurrentFunction->t.function.stackSize - size);

  } /* bJustCompute */

  return sgErr;

} /* sBE_GenDeclarationList */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenDeclaration
 */

aErr sBE_GenDeclaration(aSAST* pSAST, 
			aASTNode* pDeclaration)
{
  aErr sgErr = aErrNone;
  aASTNode* pNode;
  aASTNode* pInitDeclarator;

  aAssert(pDeclaration->eType == tDeclaration);
  pNode = pDeclaration->pChildren;

  /* there can be multiple decl specs, skip them */
  while ((pNode != NULL)
         && (pNode->eType != tInitDeclaratorList))
    pNode = pNode->pNext;

  /* walk through each init declarator in the list */
  aAssert(pNode->eType == tInitDeclaratorList);
  pInitDeclarator = pNode->pChildren;
  while ((pInitDeclarator != NULL) && (sgErr == aErrNone)) {
    aAssert(pInitDeclarator->eType == tInitDeclarator);
    sgErr = sBE_GenInitDeclarator(pSAST, pInitDeclarator);
    pInitDeclarator = pInitDeclarator->pNext;
  }

  return sgErr;

} /* sBE_GenDeclaration */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenInitDeclarator
 */

aErr sBE_GenInitDeclarator(aSAST* pSAST, 
			   aASTNode* pInitDeclarator)
{
  aErr sgErr = aErrNone;
  aASTNode* pDeclarator;
  aASTNode* pInitializer;

  aAssert(pInitDeclarator);
  aAssert(pInitDeclarator->eType == tInitDeclarator);

  /* init declarators always have a declarator as the first
   * element followed by an optional initializer */
  pDeclarator = pInitDeclarator->pChildren;
  pInitializer = pDeclarator->pNext;

  sgErr = sBE_GenDeclarator(pSAST, pDeclarator);

  /* first, handle the case where there is no initializer */
  if (sgErr == aErrNone) {
    if (pInitializer == NULL) {

      if (pSAST->bJustCompute == aTrue) {

        /* store the position of this node in the output code */
        pInitDeclarator->nCodePosition = pSAST->nCodeBytes;

        /* add in the bytes for the push literals */
        pSAST->nCodeBytes++; /* PUSHLX */
        if (pDeclarator->flags & fShort)
          pSAST->nCodeBytes += sizeof(tSHORT);
        else if (pDeclarator->flags & fByte)
          pSAST->nCodeBytes += sizeof(tBYTE);

      } else {

        /* generate code for the push literals */
        if (pDeclarator->flags & fShort) {
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHLS;
          aUtil_StoreShort(&(pSAST->code[pSAST->nCodeBytes]), 
			  (short)0xFFFF);
          pSAST->nCodeBytes += sizeof(tSHORT);
        } else if (pDeclarator->flags & fByte) {
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
          pSAST->code[pSAST->nCodeBytes++] = (char)0xFF;
        }
      }

    /* handle the initializer */
    } else {
      sgErr = sBE_GenInitializer(pSAST, pInitializer);
      
      /* mark the declarator as initialized */
      aAssert(pDeclarator->pChildren);
      aAssert(pDeclarator->pChildren->eType == tIdentifier);
      pDeclarator->pChildren->flags |= fIdentInited;
    }
  }

  /* build up the function size if we are in a function,
   * globals do not need to worry about this */  
  if (sgErr == aErrNone) {
    if (pSAST->pCurrentFunction) {
      pSAST->pCurrentFunction->t.function.stackSize += 
        (unsigned char)sTypeSizeFromFlags(pDeclarator);
    } else {
      pSAST->nStackOffset += sTypeSizeFromFlags(pDeclarator);
    }
  }

  return sgErr;

} /* sBE_GenInitDeclarator */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenInitializer
 */

aErr sBE_GenInitializer(aSAST* pSAST,
			aASTNode* pInitializer)
{
  aErr sgErr = aErrNone;
  
  aAssert(pInitializer);
  aAssert(pInitializer->eType == tInitializer);
  aAssert(pInitializer->pChildren != NULL);

  sgErr = sBE_GenExpressionProducer(pSAST, pInitializer->pChildren);

  /* we don't want any stack offset for initializers */
  if (pSAST->bJustCompute == aFalse)
    pSAST->nStackOffset = 0;

  return sgErr;

} /* sBE_GenInitializer */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenUnaryOperator
 */

aErr sBE_GenUnaryOperator(aSAST* pSAST,
			  aASTNode* pUnaryOperator)
{
  aErr sgErr = aErrNone;
  aASTNode* pPrimaryExpression;
  
  aAssert(pUnaryOperator);
  aAssert(pUnaryOperator->eType == tUnaryOperator);
  
  /* has a primary expression underneath it as a single 
   * child */
  pPrimaryExpression = pUnaryOperator->pNext;
  aAssert(pPrimaryExpression);

  /* if constant, just post the value to the stack */
  if (pUnaryOperator->flags & fConstant) {
    sgErr = sBE_GenConstant(pSAST, pUnaryOperator);

  /* else, we need to post the value below us and then 
   * apply the operator leaving the result on the stack */
  } else {

    if (pUnaryOperator->flags & fUnaryOpMinus) {

      /* push the child onto the stack */
      sgErr = sBE_GenExpressionProducer(pSAST, pPrimaryExpression);
    
      /* first, just compute the code size */
      if (pSAST->bJustCompute == aTrue) {
        pSAST->nCodeBytes += 1; /* negb or negs */
      /* now, generate the code */
      } else {
        if (pUnaryOperator->flags & fShort) {
          pSAST->code[pSAST->nCodeBytes++] = op_NEGS;
        } else if (pUnaryOperator->flags & fByte)  {
          pSAST->code[pSAST->nCodeBytes++] = op_NEGB;
        }
      }
      
    } else if (pUnaryOperator->flags & fUnaryOpPlus) {

      /* push the child onto the stack */
      sgErr = sBE_GenExpressionProducer(pSAST, pPrimaryExpression);
    
      /* first, just compute the code size */
      if (pSAST->bJustCompute == aTrue) {
        /* unary plus does nothing */
      /* now, generate the code */
      } else {
        /* unary plus does nothing */
      }
      
    } else if (pUnaryOperator->flags & fUnaryOpBang) {

      /* push the child onto the stack */
      sgErr = sBE_GenExpressionProducer(pSAST, pPrimaryExpression);
    
      /* first, just compute the code size */
      if (pSAST->bJustCompute == aTrue) {
        if (pPrimaryExpression->flags & fShort) {
          pSAST->nCodeBytes += (3 + 3 + 1 + 2 + 3 + 2);
        } else if (pPrimaryExpression->flags & fByte) {
          pSAST->nCodeBytes += (2 + 3 + 1 + 2 + 3 + 2);
        }
      /* now, generate the code */
      } else {
        /* if ==0, replace with a 1 byte */
        /* if !=0, replace with a 0 byte */
        if (pPrimaryExpression->flags & fShort) {
          /*
            pushls 0  (3)
            cmpsbr X  (3) BR = start+curr+(2)+1+2+3
            pops      (1)
            pushlb 0  (2)
            goto Z    (3) BR = start+curr+(2)+2
          X pushlb 1  (2)
          Z ...  
          */
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHLS;
          aUtil_StoreShort(&(pSAST->code[pSAST->nCodeBytes]), 
			  0x0000);
          pSAST->nCodeBytes += sizeof(tSHORT);
          pSAST->code[pSAST->nCodeBytes++] = op_CMPSBR;
          aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		            (tADDRESS)(pSAST->nCodeBytes + 
    		      		       pSAST->nCurRoutineStart + 8));
          pSAST->nCodeBytes += sizeof(tADDRESS);
          pSAST->code[pSAST->nCodeBytes++] = op_POPS;
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
          pSAST->code[pSAST->nCodeBytes++] = 0x00;
          pSAST->code[pSAST->nCodeBytes++] = op_GOTO;
          aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		            (tADDRESS)(pSAST->nCodeBytes + 
    		      		       pSAST->nCurRoutineStart + 4));
          pSAST->nCodeBytes += sizeof(tADDRESS);
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
          pSAST->code[pSAST->nCodeBytes++] = 0x01;
          
          /* ! operator converts short to byte */
          pSAST->nStackOffset -= 1;

        } else if (pPrimaryExpression->flags & fByte) {
          /*
            pushlb 0  (2)
            cmpbbr X  (3) BR = start+curr+(2)+1+2+3
            popb      (1)
            pushlb 0  (2)
            goto Z    (3) BR = start+curr+(2)+2
          X pushlb 1  (2)
          Z ...  
          */
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
          pSAST->code[pSAST->nCodeBytes++] = 0x00;
          pSAST->code[pSAST->nCodeBytes++] = op_CMPBBR;
          aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		            (tADDRESS)(pSAST->nCodeBytes + 
    		      		       pSAST->nCurRoutineStart + 8));
          pSAST->nCodeBytes += sizeof(tADDRESS);
          pSAST->code[pSAST->nCodeBytes++] = op_POPB;
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
          pSAST->code[pSAST->nCodeBytes++] = 0x00;
          pSAST->code[pSAST->nCodeBytes++] = op_GOTO;
          aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		            (tADDRESS)(pSAST->nCodeBytes + 
    		      		       pSAST->nCurRoutineStart + 4));
          pSAST->nCodeBytes += sizeof(tADDRESS);
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
          pSAST->code[pSAST->nCodeBytes++] = 0x01;
        }
      }
      
    } else if (pUnaryOperator->flags & fUnaryOpTilde) {

      /* push the child onto the stack */
      sgErr = sBE_GenExpressionProducer(pSAST, pPrimaryExpression);
    
      /* first, just compute the code size */
      if (pSAST->bJustCompute == aTrue) {
        pSAST->nCodeBytes += 1; /* compb or comps */
      /* now, generate the code */
      } else {
        if (pUnaryOperator->flags & fShort) {
          pSAST->code[pSAST->nCodeBytes++] = op_COMPS;
        } else if (pUnaryOperator->flags & fByte)  {
          pSAST->code[pSAST->nCodeBytes++] = op_COMPB;
        }
      }
      
    }

  }

  return sgErr;

} /* sBE_GenUnaryOperator */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenAssignmentExpression
 * 
 * Performs any requested assignment and leaves the result on the
 * stack.
 */

aErr sBE_GenAssignmentExpression(aSAST* pSAST,
				 aASTNode* pAssignmentExpression)
{
  aErr sgErr = aErrNone;
  aASTNode* pLValue;
  aASTNode* pAssignmentOperator;
  aASTNode* pRValue;
  
  aAssert(pAssignmentExpression);

  /* assignment expressions always start with a unary */
  pLValue = pAssignmentExpression->pChildren;
  aAssert(pLValue);
  pAssignmentOperator = pLValue->pNext;
  aAssert(pAssignmentOperator);
  pRValue = pAssignmentOperator->pNext;
  aAssert(pRValue);

  /* this puts the right hand of the equation on the stack */
  sgErr = sBE_GenExpressionProducer(pSAST, pRValue);

  /* now handle the storage (for equals) */
  if (sgErr == aErrNone)
    sgErr = sBE_GenExpressionProducer(pSAST, pLValue);

  if (sgErr == aErrNone) {
    pLValue = sGetIdentifier(pLValue);
    aAssert(pLValue);
  }
  
  /* now do the assignment */
  if (sgErr == aErrNone) 
    sgErr = sPopToIdentifierRef(pSAST, pLValue);

  return sgErr;

} /* sBE_GenAssignmentExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenUnaryExpression
 */

aErr sBE_GenUnaryExpression(aSAST* pSAST,
			    aASTNode* pUnaryExpression)
{
  aErr sgErr = aErrNone;
  aASTNode* pChild;
  
  aAssert(pUnaryExpression);
  
  pChild = pUnaryExpression->pChildren;
  aAssert(pChild);

  /* handle pre-operations, if present, leaving the result on the stack */
  if (pUnaryExpression->flags & (fUnaryPreInc | fUnaryPreDec)) {
    aASTNode* pIdentifier = pChild->pChildren;
    tSTACK stackPos;
    aAssert(pIdentifier);
    aAssert(pIdentifier->eType == tIdentifier);
    aAssert(pIdentifier->pRef);

    stackPos = pIdentifier->pRef->t.identifier.stackPos;

    /* globals are handled differently */
    if (pIdentifier->pRef->flags & fGlobal) {
      if (pSAST->bJustCompute == aTrue) {
        pSAST->nCodeBytes += 8;
      } else {
        /* get the value, do the pre-operation, put it back */
        if (pUnaryExpression->flags & fByte) {
          /* get the global onto the stack */
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHSBA;
          aTEA_StoreStack(&pSAST->code[pSAST->nCodeBytes], stackPos);
    	  pSAST->nCodeBytes += sizeof(tSTACK);
          if (pUnaryExpression->flags & fUnaryPreInc)
            pSAST->code[pSAST->nCodeBytes++] = op_INCB;
          else if (pUnaryExpression->flags & fUnaryPreDec)
            pSAST->code[pSAST->nCodeBytes++] = op_DECB;
          pSAST->code[pSAST->nCodeBytes++] = 1;
          /* put the global back */
          pSAST->code[pSAST->nCodeBytes++] = op_POPBSA;
        } else if (pUnaryExpression->flags & fShort) {
          /* get the global onto the stack */
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHSSA;
          aTEA_StoreStack(&pSAST->code[pSAST->nCodeBytes], stackPos);
    	  pSAST->nCodeBytes += sizeof(tSTACK);
          if (pUnaryExpression->flags & fUnaryPreInc)
            pSAST->code[pSAST->nCodeBytes++] = op_INCS;
          else if (pUnaryExpression->flags & fUnaryPreDec)
            pSAST->code[pSAST->nCodeBytes++] = op_DECS;
          pSAST->code[pSAST->nCodeBytes++] = 2;
          /* put the global back */
          pSAST->code[pSAST->nCodeBytes++] = op_POPSSA;
        }
        aTEA_StoreStack(&pSAST->code[pSAST->nCodeBytes], stackPos);
    	pSAST->nCodeBytes += sizeof(tSTACK);
      }
    } else {
      if (pSAST->bJustCompute == aTrue) {
        pSAST->nCodeBytes += 2;
      } else {
      
        /* set the operation */
        if (pUnaryExpression->flags & fByte) {
          if (pUnaryExpression->flags & fUnaryPreInc)
            pSAST->code[pSAST->nCodeBytes++] = op_INCB;
          else if (pUnaryExpression->flags & fUnaryPreDec)
            pSAST->code[pSAST->nCodeBytes++] = op_DECB;
        } else if (pUnaryExpression->flags & fShort) {
          if (pUnaryExpression->flags & fUnaryPreInc)
            pSAST->code[pSAST->nCodeBytes++] = op_INCS;
          else if (pUnaryExpression->flags & fUnaryPreDec)
            pSAST->code[pSAST->nCodeBytes++] = op_DECS;
        }

        /* the reference is the total stack minus 
         * the identifier's location on it
         * minus the global stack size if main and local */
        pSAST->code[pSAST->nCodeBytes++] = 
    	  (tBYTE)(pSAST->pCurrentFunction->t.function.stackSize 
    		  - stackPos
    		  + pSAST->nStackOffset
    		  );
      }
    }
  }

  /* puts the expression onto the stack */
  sgErr = sBE_GenExpressionProducer(pSAST, pChild);

  return sgErr;

} /* sBE_GenUnaryExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenPostFixExpression
 */

aErr sBE_GenPostFixExpression(aSAST* pSAST,
			      aASTNode* pPostFixExpression)
{
  aErr sgErr = aErrNone;
  aASTNode* pChild;
  
  aAssert(pPostFixExpression);
  aAssert(pPostFixExpression->eType == tPostFixExpression);
  
  pChild = pPostFixExpression->pChildren;
  aAssert(pChild);

  /* this will generate code to put the child on the stack */
  sgErr = sBE_GenExpressionProducer(pSAST, pChild);
  
  /* generate the code for increment and decrement */
  if ((sgErr == aErrNone)
      && (pPostFixExpression->flags & (fPostFixInc | fPostFixDec))) {
    tSTACK stackPos;

    aAssert(pPostFixExpression->pRef);

    stackPos = pPostFixExpression->pRef->t.identifier.stackPos;

    /* if we have a global we handle it differently */
    if (pPostFixExpression->pRef->flags & fGlobal) {
      if (pSAST->bJustCompute == aTrue) {
        pSAST->nCodeBytes += 8;
      } else {        
        if (pPostFixExpression->flags & fByte) {
          /* get a copy onto the top */
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHSBA;
          aTEA_StoreStack(&pSAST->code[pSAST->nCodeBytes], stackPos);
    	  pSAST->nCodeBytes += sizeof(tSTACK);
          /* do the math to the copy */
          if (pPostFixExpression->flags & fPostFixInc)
            pSAST->code[pSAST->nCodeBytes++] = op_INCB;
          else if (pPostFixExpression->flags & fPostFixDec)
            pSAST->code[pSAST->nCodeBytes++] = op_DECB;
          pSAST->code[pSAST->nCodeBytes++] = 1;
          /* put it back into the global's storage */
          pSAST->code[pSAST->nCodeBytes++] = op_POPBSA;
        } else if (pPostFixExpression->flags & fShort) {
          /* get a copy onto the top */
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHSSA;
          aTEA_StoreStack(&pSAST->code[pSAST->nCodeBytes], stackPos);
    	  pSAST->nCodeBytes += sizeof(tSTACK);
          /* do the math to the copy */
          if (pPostFixExpression->flags & fPostFixInc)
            pSAST->code[pSAST->nCodeBytes++] = op_INCS;
          else if (pPostFixExpression->flags & fPostFixDec)
            pSAST->code[pSAST->nCodeBytes++] = op_DECS;
          pSAST->code[pSAST->nCodeBytes++] = 2;
          /* put it back into the global's storage */
          pSAST->code[pSAST->nCodeBytes++] = op_POPSSA;
        }
        /* the global's stack reference is always the same */
        aTEA_StoreStack(&pSAST->code[pSAST->nCodeBytes], stackPos);
    	pSAST->nCodeBytes += sizeof(tSTACK);
      } /* not just computing */
    } else {
      /* all increment and decrement are 2 bytes long */
      if (pSAST->bJustCompute == aTrue) {
        pSAST->nCodeBytes += 2;
      } else {

        /* set the operation */
        if (pPostFixExpression->flags & fByte) {
          if (pPostFixExpression->flags & fPostFixInc)
            pSAST->code[pSAST->nCodeBytes++] = op_INCB;
          else if (pPostFixExpression->flags & fPostFixDec)
            pSAST->code[pSAST->nCodeBytes++] = op_DECB;
        } else if (pPostFixExpression->flags & fShort) {
          if (pPostFixExpression->flags & fPostFixInc)
            pSAST->code[pSAST->nCodeBytes++] = op_INCS;
          else if (pPostFixExpression->flags & fPostFixDec)
            pSAST->code[pSAST->nCodeBytes++] = op_DECS;
        }

        /* the reference is the total stack minus 
         * the identifier's location on it */
        pSAST->code[pSAST->nCodeBytes++] = 
    	  (tBYTE)(pSAST->pCurrentFunction->t.function.stackSize
    		  - stackPos
    		  + pSAST->nStackOffset
    		  );
      } /* not just computing */
    }
  }

  return sgErr;

} /* sBE_GenPostFixExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenPrimaryExpression
 */

aErr sBE_GenPrimaryExpression(aSAST* pSAST,
			      aASTNode* pPrimaryExpression)
{
  aErr sgErr = aErrNone;
  aASTNode* pChild;

  aAssert(pPrimaryExpression);
  aAssert(pPrimaryExpression->eType == tPrimaryExpression);
  
  pChild = pPrimaryExpression->pChildren;
  aAssert(pChild);

  switch (pChild->eType) {

  case tConstant:
    sgErr = sBE_GenConstant(pSAST, pChild);
    break;

  case tString:
    sgErr = sBE_GenString(pSAST, pChild);
    break;

  case tIdentifier:
    sgErr = sBE_GenIdentifier(pSAST, pChild);
    break;

  case tExpression:
    sgErr = sBE_GenExpression(pSAST, pChild);
    break;
  
  default:
    aAssert(0); /* should we get here ? */
    break;

  } /* switch */

  return sgErr;

} /* sBE_GenPrimaryExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenConstant
 */

aErr sBE_GenConstant(aSAST* pSAST,
		     aASTNode* pConstant)
{
  aErr sgErr = aErrNone;
  
  aAssert(pConstant);
  aAssert(pConstant->flags & fConstant);

  /* we only output if we are told to */
  if (pConstant->flags & fPushVal) {

    if (pSAST->bJustCompute == aTrue) {

     /* compute the size of the push literals */
      if (pConstant->flags & fShort) {
        pSAST->nCodeBytes += 3;
      } else if (pConstant->flags & fByte) {
        pSAST->nCodeBytes += 2;
      }

    } else {

      /* generate code for the push literals */
      if (pConstant->flags & fShort) {
        pSAST->code[pSAST->nCodeBytes++] = op_PUSHLS;
        aUtil_StoreShort(&(pSAST->code[pSAST->nCodeBytes]), 
			pConstant->v.shortVal);
        pSAST->nCodeBytes += 2;
      } else if (pConstant->flags & fByte) {
        pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
        pSAST->code[pSAST->nCodeBytes++] = pConstant->v.byteVal;
      }

      /* now account for the constant on the stack */
      pSAST->nStackOffset += sTypeSizeFromFlags(pConstant);
    }
  }

  return sgErr;

} /* sBE_GenConstant */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenString
 */

aErr sBE_GenString(aSAST* pSAST,
		   aASTNode* pString)
{
  aErr sgErr = aErrNone;
  
  aAssert(pString);
  aAssert(pString->flags & fConstant);
  aAssert(pString->pToken);
  aAssert(pString->pToken->eType == tkString);

  /* we only output if we are told to */
  if (pString->flags & fPushVal) {

    if (pSAST->bJustCompute == aTrue) {

     /* two bytes per character plus the length */
     pSAST->nCodeBytes += 
     	(tADDRESS)((pString->t.string.length + 1) * 2);

    } else {
      unsigned int i;
      unsigned int len = pString->t.string.length;
      char* p = pString->pToken->v.string;
      
      /* generate code for pushing the string */
      for (i = 0; i < len; i++, p++) {
        pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
        pSAST->code[pSAST->nCodeBytes++] = *p;
      }
      
      /* add the length byte at the end */
      pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
      pSAST->code[pSAST->nCodeBytes++] = (tBYTE)len;

      /* now account for the constant on the stack */
      pSAST->nStackOffset += (tADDRESS)(len + 1);
    }
  }

  return sgErr;

} /* sBE_GenString */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenDeclarator
 */

aErr sBE_GenDeclarator(aSAST* pSAST,
		       aASTNode* pDeclarator)
{
  aErr sgErr = aErrNone;
  aASTNode* pIdentifier;
  aASTNode* pParameterList;

  aAssert(pDeclarator);
  aAssert(pDeclarator->eType == tDeclarator);

  if (pSAST->bJustCompute == aTrue) {

    /* store the position of this node in the output code */
    pDeclarator->nCodePosition = pSAST->nCodeBytes;

    /* all declarators start with an identifier */
    pIdentifier = pDeclarator->pChildren;
    if (pDeclarator->pParent->eType != tFunctionDefinition)
      sgErr = sBE_GenIdentifier(pSAST, pIdentifier);

    /* for functions, store the routine address (absolute) in 
     * the identifier's code position since the function calls 
     * will point to this and the function's code position 
     * holds the total routine size
     */
    else
      pIdentifier->nCodePosition = pSAST->nCurRoutineStart;
    if (sgErr == aErrNone) {
      /* check for a parameter list following */
      pParameterList = pIdentifier->pNext;
      if (pParameterList != NULL)
        sgErr = sBE_GenParameterList(pSAST, pParameterList);
    } 
  }

  return sgErr;

} /* sBE_GenDeclarator */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenIdentifier
 */

aErr sBE_GenIdentifier(aSAST* pSAST,
		       aASTNode* pIdentifier)
{
  aErr sgErr = aErrNone;
  aASTNode* pArgument;
  aASTNode* pArgumentList; 

  aAssert(pIdentifier);
  aAssert(pIdentifier->pParent);

  if (pSAST->bJustCompute == aTrue) {

    /* store the position of this node in the output code */
    pIdentifier->nCodePosition = (tADDRESS)(pSAST->nCodeBytes +
    					    pSAST->nCurRoutineStart);

    /* add this in to the current function's stack size
     * and also point the identifier to the function
     */
    if (pIdentifier->pParent->eType == tDeclarator) {
      if (pSAST->pCurrentFunction) {
        pIdentifier->pRef = pSAST->pCurrentFunction;
        pIdentifier->t.identifier.stackPos = 
      	  pSAST->pCurrentFunction->t.function.stackSize;
      } else {
        pIdentifier->t.identifier.stackPos = 
      	  (unsigned char)pSAST->nStackOffset;
      }

    /* it is a routine call */
    } else if (pIdentifier->flags & fAddress) {

      /* first, account for the return value (PUSHN size) */
      if (!(pIdentifier->flags & fVoid))
        pSAST->nCodeBytes += 2;

      /* next, size the arguments pushed onto the stack */
      aAssert(pIdentifier->pParent);
      pArgumentList = pIdentifier->pParent->pNext;
      aAssert(pArgumentList);
      pArgumentList->t.argumentList.stackSize = 0;
      pArgument = pArgumentList->pChildren;
      while ((sgErr == aErrNone) && pArgument) {
        pArgumentList->t.argumentList.stackSize += 
        	sTypeSizeFromFlags(pArgument);
        sgErr = sBE_GenExpressionProducer(pSAST, pArgument);
        pArgument = pArgument->pNext;
      }

      /* the routine call */
      pSAST->nCodeBytes += 3;

      /* clean up any parameters with some type of pop */
      if (pArgumentList->t.argumentList.stackSize > 2)
        pSAST->nCodeBytes += 2; /* popn */
      else if (pArgumentList->t.argumentList.stackSize > 0)
        pSAST->nCodeBytes += 1;  /* popx */

      /* and clean up after ourselves if not pushing a value */
      if ((sgErr == aErrNone)
          && !(pIdentifier->flags & fPushVal))
        sgErr = sPopExpression(pSAST, pIdentifier);

    /* it is a simple expression */
    } else if (pIdentifier->flags & fPushVal) {
      sgErr = sPushFromIdentifierRef(pSAST, pIdentifier);
    }

  /* code generation */
  } else {

    /* routine call */  
    if (pIdentifier->flags & fAddress) {

      /* first, build space for the return value */
      if (!(pIdentifier->flags & fVoid)) {
        tBYTE size = sTypeSizeFromFlags(pIdentifier);
        pSAST->code[pSAST->nCodeBytes++] = op_PUSHN;
        pSAST->code[pSAST->nCodeBytes++] = size;
        /* now account for this on the stack */
        pSAST->nStackOffset += size;
      }

      /* next, size the arguments pushed onto the stack */
      aAssert(pIdentifier->pParent);
      pArgumentList = pIdentifier->pParent->pNext;
      aAssert(pArgumentList);
      pArgument = pArgumentList->pChildren;
      while ((sgErr == aErrNone) && pArgument) {
        sgErr = sBE_GenExpressionProducer(pSAST, pArgument);
        pArgument = pArgument->pNext;
      }

      /* the routine call */
      pSAST->code[pSAST->nCodeBytes++] = op_CALL;
      aAssert(pIdentifier->pRef);
      aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
          		pIdentifier->pRef->nCodePosition);
      pSAST->nCodeBytes += sizeof(tADDRESS);
 
      pSAST->nStackOffset -= pArgumentList->t.argumentList.stackSize;
      if (pArgumentList->t.argumentList.stackSize > 2) {
        pSAST->code[pSAST->nCodeBytes++] = op_POPN;
        pSAST->code[pSAST->nCodeBytes++] = 
        	pArgumentList->t.argumentList.stackSize;
      } else if (pArgumentList->t.argumentList.stackSize == 2) {
        pSAST->code[pSAST->nCodeBytes++] = op_POPS;
      } else if (pArgumentList->t.argumentList.stackSize == 1) {
        pSAST->code[pSAST->nCodeBytes++] = op_POPB;
      }
 
      /* and clean up after ourselves if not pushing a value */
      if ((sgErr == aErrNone)
          && !(pIdentifier->flags & fPushVal))
        sgErr = sPopExpression(pSAST, pIdentifier);

    /* pushing a simple identifier */
    } else if (pIdentifier->flags & fPushVal) {
      sgErr = sPushFromIdentifierRef(pSAST, pIdentifier);
    }
  }

  return sgErr;

} /* sBE_GenIdentifier */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenParameterList 
 */

aErr sBE_GenParameterList(aSAST* pSAST,
			  aASTNode* pParameterList)
{
  aErr sgErr = aErrNone;
  aASTNode* pParameterDeclaration;

  if (pSAST->bJustCompute == aTrue) {
    unsigned char initialSize;
    aAssert(pParameterList);
    aAssert(pSAST->pCurrentFunction);
    aAssert(pParameterList->eType == tParameterList);

    /* store the position of this node in the output code */
    pParameterList->nCodePosition = pSAST->nCodeBytes;

    /* we store the total parameter list size in the 
     * parameter list node's byte value */
    aAssert(pSAST->pCurrentFunction);
    initialSize = pSAST->pCurrentFunction->t.function.stackSize;

    pParameterDeclaration = pParameterList->pChildren;

    /* add on each parameter */
    while ((pParameterDeclaration != NULL)
           && (sgErr == aErrNone)) {
      aASTNode* pChild = pParameterDeclaration->pChildren;
      aAssert(pParameterDeclaration->eType == tParameterDeclaration);

      aAssert(pChild);
      aAssert(pChild->eType == tDeclarationSpecifier);

      /* stick a reference to the function on the declarator
       * so it can figure out it's stack depth */
      sgErr = sBE_GenDeclarator(pSAST, pChild->pNext);

      /* add this parameter to the stack */
      pSAST->pCurrentFunction->t.function.stackSize += 
        (unsigned char)sTypeSizeFromFlags(pChild->pNext);

      pParameterDeclaration = pParameterDeclaration->pNext;
    }

    /* now store the parameter list size */
    pParameterList->t.parameterList.stackSize = 
    	(unsigned char)(pSAST->pCurrentFunction->t.function.stackSize 
    			- initialSize);

  } /* bJustCompute */

  return sgErr;

} /* sBE_GenParameterList */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenStatementList 
 */

aErr sBE_GenStatementList(aSAST* pSAST,
			  aASTNode* pStatementList)
{
  aErr sgErr = aErrNone;
  aASTNode* pStatement;
  
  aAssert(pStatementList);
  aAssert(pStatementList->eType == tStatementList);

  pStatement = pStatementList->pChildren;
  while ((pStatement != NULL) 
         && (sgErr == aErrNone)) {
    sgErr = sBE_GenStatement(pSAST, pStatement);
    pStatement = pStatement->pNext;
  }

  return sgErr;

} /* sBE_GenStatementList */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenStatement
 */

aErr sBE_GenStatement(aSAST* pSAST,
		      aASTNode* pStatement)
{
  aErr sgErr = aErrNone;
  aASTNode* pChild;

  aAssert(pStatement);
  aAssert(pStatement->eType == tStatement);

  pChild = pStatement->pChildren;
  aAssert(pChild);

  switch (pChild->eType) {
  case tExpressionStatement:
    sgErr = sBE_GenExpressionStatement(pSAST, pChild);
    break;
  case tAsmStatement:
    sgErr = sBE_GenAsmStatement(pSAST, pChild);
    break;
  case tJumpStatement:
    sgErr = sBE_GenJumpStatement(pSAST, pChild);
    break;
  case tLabeledStatement:
    sgErr = sBE_GenLabeledStatement(pSAST, pChild);
    break;
  case tSelectionStatement:
    sgErr = sBE_GenSelectionStatement(pSAST, pChild);
    break;
  case tCompoundStatement:
    sgErr = sBE_GenCompoundStatement(pSAST, pChild);
    break;
  case tLoopStatement:
    sgErr = sBE_GenLoopStatement(pSAST, pChild);
    break;
  default:
    aAssert(0); /* should we get here ? */
    break;
  } /* switch */

  return sgErr;

} /* sBE_GenStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenAsmStatement
 */

aErr sBE_GenAsmStatement(aSAST* pSAST,
			 aASTNode* pAsmStatement)
{
  aErr sgErr = aErrNone;
  aASTNode* pAsmList;
  aASTNode* pChild;

  aAssert(pAsmStatement);
  aAssert(pAsmStatement->eType == tAsmStatement);

  pAsmList = pAsmStatement->pChildren;
  aAssert(pAsmList);
  
  pChild = pAsmList->pChildren;
  
  while ((pChild != NULL)
         && (sgErr == aErrNone)) {

    switch (pChild->eType) {

    case tOpcode:
      sgErr = sBE_GenOpcode(pSAST, pChild);
      break;

    case tLabeledStatement:
      sgErr = sBE_GenLabeledStatement(pSAST, pChild);
      break;

    default:
      aAssert(0); /* should we get here ? */
      break;

    } /* switch */

    pChild = pChild->pNext;

  } /* while */

  return sgErr;

} /* sBE_GenAsmStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenOpcode
 */

aErr sBE_GenOpcode(aSAST* pSAST,
		   aASTNode* pOpcode)
{
  aErr sgErr = aErrNone;

  aAssert(pOpcode);
  aAssert(pOpcode->eType == tOpcode);

  if (pSAST->bJustCompute) {

    /* store the position of this node in the output code */
    pOpcode->nCodePosition = pSAST->nCodeBytes;

    pSAST->nCodeBytes += pOpcode->t.opcode.asmCode.opLen;

  } else {
    /* if the opcode takes an address, add in the offset 
     * from the beginning of the routine */
    if (aTEA_OpCodeTakesAddress((tOpCode)pOpcode->t.opcode.asmCode.code[0])) {
      aASTNode* pIdentifier = pOpcode->pChildren;
      aAssert(pIdentifier);
      aAssert(pIdentifier->pRef);
      aAssert(pIdentifier->flags & fAddress);
      aTEA_StoreAddress(&pOpcode->t.opcode.asmCode.code[1], 
          		pIdentifier->pRef->nCodePosition);
    }
    aMemCopy(&pSAST->code[pSAST->nCodeBytes],
    	     &pOpcode->t.opcode.asmCode.code,
    	     pOpcode->t.opcode.asmCode.opLen);
    pSAST->nCodeBytes += pOpcode->t.opcode.asmCode.opLen;
  }

  return sgErr;

} /* sBE_GenOpcode */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenJumpStatement
 */

aErr sBE_GenJumpStatement(aSAST* pSAST,
			  aASTNode* pJumpStatement)
{
  aErr sgErr = aErrNone;
  unsigned char localStackBytes;

  aAssert(pJumpStatement);
  aAssert(pJumpStatement->eType == tJumpStatement);

  if (pSAST->bJustCompute) {

    /* store the position of this node in the output code */
    pJumpStatement->nCodePosition = pSAST->nCodeBytes;

    /* cache the current function for return statements */
    if (pJumpStatement->flags & fJumpReturn) {
      pJumpStatement->pRef = pSAST->pCurrentFunction;

      /* handle the return value */
      if (!(pJumpStatement->pRef->flags & fVoid)) {
        aAssert(pJumpStatement->pChildren);
        sgErr = sBE_GenExpression(pSAST, pJumpStatement->pChildren);

        /* the pop back into the stack of the return value 
         * and the return statement */
        pSAST->nCodeBytes += 2;
      }

      /* handle cleaning up the local stack */
      if (sgErr == aErrNone) {
        aAssert(pSAST->pCurrentFunction->t.function.stackSize >=
                pSAST->pCurrentFunction->t.function.callSize);
        localStackBytes = 
        	(unsigned char)(pSAST->pCurrentFunction->t.function.stackSize 
        			- pSAST->pCurrentFunction->t.function.callSize);
        if (localStackBytes > 2)
          pSAST->nCodeBytes += 2;
        else if (localStackBytes > 0)
          pSAST->nCodeBytes += 1;
      }

      /* now handle the exit statement */
      pSAST->nCodeBytes += 1;
    } else {
      /* continue, goto, and break all are simple goto opcodes */
      pSAST->nCodeBytes += 3; /* for the goto */
    }
  } else {
 
    /* if there is a return value, pop it into
     * the stack */
    if (pJumpStatement->flags & fJumpReturn) {
 
      /* ref should hold function */
      aAssert(pJumpStatement->pRef);

      /* generate the code that will leave the expression on the stack */    
      if (pJumpStatement->pChildren) {
        aASTNode* pExpression = pJumpStatement->pChildren;
        sgErr = sBE_GenExpression(pSAST, pExpression);

        /* pop the return value into the return value location */
        if (sgErr == aErrNone) {
        
          if (pExpression->flags & fShort) {
            pSAST->code[pSAST->nCodeBytes++] = op_POPSS;
            pSAST->nStackOffset -= sizeof(tSHORT);
          } else if (pExpression->flags & fByte) {
            pSAST->code[pSAST->nCodeBytes++] = op_POPBS;
            pSAST->nStackOffset -= sizeof(tBYTE);
          }

          /* the function holds the complete stack call size */
          /* (must subtract global stack offset if in main) */
          pSAST->code[pSAST->nCodeBytes++] = 
          	(char)(pJumpStatement->pRef->t.function.stackSize);
        }
      }
 
      /* pop off any local stack variables */
      if (sgErr == aErrNone) {
        aAssert(pSAST->pCurrentFunction->t.function.stackSize >=
                pSAST->pCurrentFunction->t.function.callSize);
        localStackBytes = 
        	(unsigned char)(pSAST->pCurrentFunction->t.function.stackSize 
        			- pSAST->pCurrentFunction->t.function.callSize);
        if (localStackBytes > 2) {
          pSAST->code[pSAST->nCodeBytes++] = op_POPN;
          pSAST->code[pSAST->nCodeBytes++] = (char)localStackBytes;
        } else if (localStackBytes == 2) {
          pSAST->code[pSAST->nCodeBytes++] = op_POPS;
        } else if (localStackBytes == 1) {
          pSAST->code[pSAST->nCodeBytes++] = op_POPB;
        }
      }

      /* write out the return */
      if (sgErr == aErrNone)
        pSAST->code[pSAST->nCodeBytes++] = op_RETURN;

    /* handle break statements */
    } else if (pJumpStatement->flags & fJumpBreak) {
      /* the current breakable block is held in the ref */
      aASTNode* pLoop = pJumpStatement->pRef;
      aAssert(pLoop);
      aAssert(pLoop->flags & fBreakable);

      /* break and continue are gotos */
      pSAST->code[pSAST->nCodeBytes++] = op_GOTO;

      aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes],
           		pLoop->t.loop.loopEnd);
      pSAST->nCodeBytes += sizeof(tADDRESS);

    /* handle continue statements */
    } else if (pJumpStatement->flags & fJumpContinue) {
      /* the current outer loop is held in the ref */
      aASTNode* pLoop = pJumpStatement->pRef;
      aAssert(pLoop);
      aAssert(pLoop->eType == tLoopStatement);

      /* break and continue are gotos */
      pSAST->code[pSAST->nCodeBytes++] = op_GOTO;

      aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes],
           		pLoop->t.loop.loopContinue);
      pSAST->nCodeBytes += sizeof(tADDRESS);

    /* handle goto statements */
    } else if (pJumpStatement->flags & fJumpGoto) {
      aASTNode* pIdentifier = pJumpStatement->pChildren;
      aAssert(pIdentifier);
      aAssert(pIdentifier->eType == tIdentifier);
      aAssert(pIdentifier->flags & fConstant);
      aAssert(pIdentifier->flags & fAddress);
      aAssert(pIdentifier->pRef);
      aAssert(pIdentifier->pRef->eType == tLabeledStatement);
      pSAST->code[pSAST->nCodeBytes++] = op_GOTO;
      aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes],
           		pIdentifier->pRef->nCodePosition);
      pSAST->nCodeBytes += sizeof(tADDRESS);
    }
  }
  
  return sgErr;

} /* sBE_GenJumpStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenExpression
 */

aErr sBE_GenExpression(aSAST* pSAST,
		       aASTNode* pExpression)
{
  aErr sgErr = aErrNone;
  aASTNode* pChild;
  
  aAssert(pExpression);
  aAssert(pExpression->eType == tExpression);

  pChild = pExpression->pChildren;

  while ((pChild != NULL) && (sgErr == aErrNone)) {
    sgErr = sBE_GenExpressionProducer(pSAST, pChild);
    pChild = pChild->pNext;
  }

  return sgErr;

} /* sBE_GenExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenExpressionStatement
 */

aErr sBE_GenExpressionStatement(aSAST* pSAST,
		       		aASTNode* pExpressionStatement)
{
  aErr sgErr = aErrNone;
  aASTNode* pExpression;

  aAssert(pExpressionStatement);
  aAssert(pExpressionStatement->eType == tExpressionStatement);

  pExpression = pExpressionStatement->pChildren;

  while ((pExpression != NULL)
         && (sgErr == aErrNone)) {
    sgErr = sBE_GenExpression(pSAST, pExpression);
    pExpression = pExpression->pNext;
  }

  return sgErr;

} /* sBE_GenExpressionStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenLabeledStatement
 */

aErr sBE_GenLabeledStatement(aSAST* pSAST,
			     aASTNode* pLabeledStatement)
{
  aErr sgErr = aErrNone;

  aAssert(pLabeledStatement);
  aAssert(pLabeledStatement->eType == tLabeledStatement);
  
  /* store where we are in the code so others can reference
   * the spot */
  if (pSAST->bJustCompute)
    pLabeledStatement->nCodePosition = 
    	(tADDRESS)(pSAST->nCodeBytes + pSAST->nCurRoutineStart);

  return sgErr;

} /* sBE_GenLabeledStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenSelectionStatement
 */

aErr sBE_GenSelectionStatement(aSAST* pSAST,
			       aASTNode* pSelectionStatement)
{
  aErr sgErr = aErrNone;
  aASTNode* pExpression;
  aASTNode* pStatement = NULL;
  tADDRESS mark1 = 0;
  tADDRESS mark2 = 0;

  aAssert(pSelectionStatement);
  aAssert(pSelectionStatement->eType == tSelectionStatement);

  pExpression = pSelectionStatement->pChildren;
  aAssert(pExpression);

  /* store where we are in the code so others can reference
   * the spot */
  if (pSAST->bJustCompute)
    pSelectionStatement->nCodePosition = pSAST->nCodeBytes;

  /* handle the expression code */
  if (!(pExpression->flags & fConstant)) {
    sgErr = sBE_GenExpression(pSAST, pExpression);

    /* handle the actual selection code */
    if ((sgErr == aErrNone) 
        && (pSelectionStatement->flags & fSelectIf)) {
      if (pSAST->bJustCompute) {

	/* pop off the expression result */
        sgErr = sPopExpression(pSAST, pExpression);

        /* we do a BRZ so account for them now */
        pSAST->nCodeBytes += 3;
      } else {

	/* pop off the expression result */
	sgErr = sPopExpression(pSAST, pExpression);

        /* now, check for zero and branch */
        pSAST->code[pSAST->nCodeBytes++] = op_BRZ;
        /* save this position, we will fill it in after the statement */
        mark1 = pSAST->nCodeBytes;
        pSAST->nCodeBytes += 2;
      }

    /* handle the switch selection */
    } else if ((sgErr == aErrNone) 
               && (pSelectionStatement->flags & fSelectSwitch)) {
      aASTNode* pChain = pSelectionStatement->t.switchChain.pNext;
      aASTNode* pLast = NULL;
      /* chain points to the case and default labels */
      while ((sgErr == aErrNone) && pChain) {
        if (pChain->flags & fLabelCase) {
          aASTNode *pConstantExpression = pChain->pChildren;
          sgErr = sBE_GenExpressionProducer(pSAST, pConstantExpression);
          if (sgErr == aErrNone) {
            if (pSAST->bJustCompute)
              pSAST->nCodeBytes += 3; /* cmpXbr */
            else {
              pSAST->nStackOffset -= sTypeSizeFromFlags(pConstantExpression);
              if (pConstantExpression->flags & fByte)
                pSAST->code[pSAST->nCodeBytes++] = op_CMPBBR;
              else if (pConstantExpression->flags & fShort)
                pSAST->code[pSAST->nCodeBytes++] = op_CMPSBR;
              aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
          		        pChain->nCodePosition);
      	      pSAST->nCodeBytes += sizeof(tADDRESS);
            }
          }
        } else if (pChain->flags & fLabelDefault) {
          /* pop off the expression */
          sgErr = sPopExpression(pSAST, pExpression);
          if (pSAST->bJustCompute) {
            pSAST->nCodeBytes += 3; /* goto */
          } else {
            pSAST->code[pSAST->nCodeBytes++] = op_GOTO;
            aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
          		      pChain->nCodePosition);
      	    pSAST->nCodeBytes += sizeof(tADDRESS);
          }
        }
        pLast = pChain;
        pChain = pChain->t.switchChain.pNext;
      }
      
      /* if we are falling off the end, we need to clean up */
      if (!pLast || !(pLast->flags & fLabelDefault)) {
        /* pop off the expression */
        sgErr = sPopExpression(pSAST, pExpression);
        if (pSAST->bJustCompute) {
          pSAST->nCodeBytes += 3; /* goto */
        } else {
          pSAST->code[pSAST->nCodeBytes++] = op_GOTO;
          aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
          		    pSelectionStatement->t.switchChain.switchEnd);
      	  pSAST->nCodeBytes += sizeof(tADDRESS);
      	}
      }
    }
  }

  /* now compute the conditional statement */
  if ((sgErr == aErrNone)
      && (!(pExpression->flags & fConstant)
          || ((pExpression->flags & fByte)
              && pExpression->v.byteVal)
          || ((pExpression->flags & fShort)
              && pExpression->v.shortVal))) {
    pStatement = pExpression->pNext;
    aAssert(pStatement);
    sgErr = sBE_GenStatement(pSAST, pStatement);
  }

  /* now that the statement is output, install the jump address */
  if ((sgErr == aErrNone) 
      && (pSelectionStatement->flags & fSelectIf)
      && !(pExpression->flags & fConstant)) {

    if (pSAST->bJustCompute == aTrue) {

      /* handle the else clause here (GOTO) */
      if ((pSelectionStatement->flags & fSelectIf)
          && (pStatement->pNext != NULL)) {
        pSAST->nCodeBytes += 3;
        sgErr = sBE_GenStatement(pSAST, pStatement->pNext);
      }

    } else {

      /* handle the else clause here (GOTO) */
      if ((pSelectionStatement->flags & fSelectIf)
          && (pStatement->pNext != NULL)) {
        pSAST->code[pSAST->nCodeBytes++] = op_GOTO;
        /* save this mark for later */
        mark2 = pSAST->nCodeBytes;
        pSAST->nCodeBytes += 2;
      }

      aTEA_StoreAddress(&pSAST->code[mark1], 
      			(tADDRESS)(pSAST->nCodeBytes 
      				   + pSAST->nCurRoutineStart));

      if ((pSelectionStatement->flags & fSelectIf)
          && (pStatement->pNext != NULL)) {
        sgErr = sBE_GenStatement(pSAST, pStatement->pNext);
        if (sgErr == aErrNone) {
          aTEA_StoreAddress(&pSAST->code[mark2], 
          		    (tADDRESS)(pSAST->nCodeBytes 
          		               + pSAST->nCurRoutineStart));
        }
      }
    }
  }

  if ((sgErr == aErrNone)
      && (pSelectionStatement->flags & fSelectSwitch)) {
    
    /* when the statement is generating code, it needs to know where
     * the break statements end up */
    if (pSAST->bJustCompute)
      pSelectionStatement->t.switchChain.switchEnd = 
      	(tADDRESS)(pSAST->nCurRoutineStart + pSAST->nCodeBytes);
  }

  return sgErr;

} /* sBE_GenSelectionStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenLoopStatement
 */

aErr sBE_GenLoopStatement(aSAST* pSAST,
			  aASTNode* pLoopStatement)
{
  aErr sgErr = aErrNone;
  aASTNode* pPreExpr = NULL;
  aASTNode* pLoopExpr = NULL;
  aASTNode* pPostExpr = NULL;
  aASTNode* pStatement;
  tADDRESS doneJump = 0;

  aAssert(pLoopStatement);
  aAssert(pLoopStatement->eType == tLoopStatement);

  pStatement = pLoopStatement->pChildren;
  aAssert(pStatement);
 
  /* figure out what children we have to generate code for */
  while (pStatement->eType != tStatement) {
    if (pStatement->flags & fLoopPre) {
      pPreExpr = pStatement;
    } else if (pStatement->flags & fLoopCondition) {
      pLoopExpr = pStatement;
    } else if (pStatement->flags & fLoopPost) {
      pPostExpr = pStatement;
    }
    pStatement = pStatement->pNext;
  }

  /* store where we are in the code so others can reference
   * the spot */
  if (pSAST->bJustCompute)
    pLoopStatement->nCodePosition = pSAST->nCodeBytes;

  /* first, generate the pre-expression if we have one */
  if (pPreExpr)
    sgErr = sBE_GenExpression(pSAST, pPreExpr);

  /* now, store a mark at the loop start */
  if ((sgErr == aErrNone)
      && (pSAST->bJustCompute == aFalse))
    pLoopStatement->t.loop.loopEval = 
      	(tADDRESS)(pSAST->nCurRoutineStart + pSAST->nCodeBytes);

  /* handle the expression if not a do-while loop */
  if (pLoopExpr && !(pLoopStatement->flags & fLoopDoWhile)) {

    /* generate the expression on the stack */
    sgErr = sBE_GenExpression(pSAST, pLoopExpr);

    /* pop off the expression result from the stack */
    if (sgErr == aErrNone)
      sgErr = sPopExpression(pSAST, pLoopExpr);

    /* perform the expression test */
    if (sgErr == aErrNone) {
      if (pSAST->bJustCompute) {
        /* we do a BRZ so account for it now */
        pSAST->nCodeBytes += 3;
      } else {
        /* now, check for zero and branch */
        pSAST->code[pSAST->nCodeBytes++] = op_BRZ;
        /* save this position, we will fill it in later */
        doneJump = pSAST->nCodeBytes;
        pSAST->nCodeBytes += sizeof(tADDRESS);
      }
    }
  }

  /* now generate the loop statement which everybody has */
  if (sgErr == aErrNone) {
    aAssert(pStatement);
    sgErr = sBE_GenStatement(pSAST, pStatement);
  }

  /* (MRW) when the statement is generating code, it needs to know
   * where continue statements might go */
  if (pSAST->bJustCompute) {
    pLoopStatement->t.loop.loopContinue = 
      	(tADDRESS)(pSAST->nCurRoutineStart + pSAST->nCodeBytes);
  }

  /* now, if there was a post-condition, generate it */
  if (pPostExpr && (sgErr == aErrNone))
    sgErr = sBE_GenExpression(pSAST, pPostExpr);

  /* generate the test for the do-while construct */
  if (pLoopExpr && (pLoopStatement->flags & fLoopDoWhile)) {
    /* generate the expression on the stack */
    sgErr = sBE_GenExpression(pSAST, pLoopExpr);
    /* pop off the expression result from the stack */
    if (sgErr == aErrNone)
      sgErr = sPopExpression(pSAST, pLoopExpr);
    /* perform the expression test */
    if (sgErr == aErrNone) {
      if (pSAST->bJustCompute) {
        /* we do a BRZ so account for it now */
        pSAST->nCodeBytes += 3;
      } else {
        /* now, check for zero and branch */
        pSAST->code[pSAST->nCodeBytes++] = op_BRZ;
        /* save this position, we will fill it in later */
        doneJump = pSAST->nCodeBytes;
        pSAST->nCodeBytes += sizeof(tADDRESS);
      }
    }
  }

  /* now install the loop construct */
  if (sgErr == aErrNone) {
    if (pSAST->bJustCompute == aTrue) {
      /* account for the loop back */
      pSAST->nCodeBytes += 3;
    } else {
      /* do the loop back */
      pSAST->code[pSAST->nCodeBytes++] = op_GOTO;
      aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
        		pLoopStatement->t.loop.loopEval);
      pSAST->nCodeBytes += sizeof(tADDRESS);

      /* patch up the forward loop exit position */
      aTEA_StoreAddress(&pSAST->code[doneJump], 
      			(tADDRESS)(pSAST->nCodeBytes 
      			+ pSAST->nCurRoutineStart));
    }
  }

  /* when the statement is generating code, it needs to know where
   * the break statements end up */
  if (pSAST->bJustCompute) {
    aAssert(pLoopStatement->flags & fBreakable);
    pLoopStatement->t.loop.loopEnd = 
      	(tADDRESS)(pSAST->nCurRoutineStart + pSAST->nCodeBytes);
  }

  return sgErr;

} /* sBE_GenLoopStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenCompoundStatement
 */

aErr sBE_GenCompoundStatement(aSAST* pSAST,
			      aASTNode* pCompoundStatement)
{
  aErr sgErr = aErrNone;
  aASTNode* pDeclarationList = NULL;
  aASTNode* pStatementList = NULL;
  
  aAssert(pSAST);
  aAssert(pCompoundStatement);
  aAssert(pCompoundStatement->eType == tCompoundStatement);

  if (pCompoundStatement->pChildren) {
    if (pCompoundStatement->pChildren->eType == tDeclarationList) {
      pDeclarationList = pCompoundStatement->pChildren;
      pStatementList = pDeclarationList->pNext;
    } else if (pCompoundStatement->pChildren->eType == tStatementList) {
      pStatementList = pCompoundStatement->pChildren;
    }
  }

  /* handle the declaration list, if present */
  if (pDeclarationList) {
    aAssert(pDeclarationList->eType == tDeclarationList);
    sgErr = sBE_GenDeclarationList(pSAST, pDeclarationList);
  }

  /* handle the statement list, if present */  
  if (pStatementList && (sgErr == aErrNone)) {
    aAssert(pStatementList->eType == tStatementList);
    sgErr = sBE_GenStatementList(pSAST, pStatementList);
  }

  /* now, we need to handle cleaning up the declarations at
   * the head of this compound statement */
  if (pDeclarationList && (sgErr == aErrNone)) {

    /* since the main compound statement in a routine requires 
     * a return statement at the end, we don't need to clean up
     * the routine's local variables since the return does this
     * for us.  All other compound statements's local variable
     * stack storage needs to be cleaned up */
    if (pCompoundStatement->pParent->eType != tFunctionDefinition) {
      if (pSAST->bJustCompute) {

         /* compute the size of the pop commands */
         if (pDeclarationList->t.declarationList.stackSize > 2)
           pSAST->nCodeBytes += 2;
         else if (pDeclarationList->t.declarationList.stackSize > 0)
           pSAST->nCodeBytes += 1;

      } else {

        /* implement the size of the pop commands */
        if (pDeclarationList->t.declarationList.stackSize > 2) {
          pSAST->code[pSAST->nCodeBytes++] = op_POPN;
          pSAST->code[pSAST->nCodeBytes++] = 
		(char)pDeclarationList->t.declarationList.stackSize;
        } else if (pDeclarationList->t.declarationList.stackSize == 2) {
          pSAST->code[pSAST->nCodeBytes++] = op_POPS;
        } else if (pDeclarationList->t.declarationList.stackSize == 1) {
          pSAST->code[pSAST->nCodeBytes++] = op_POPB;
        }

      }
    }

    /* either way, reduce the size of the routine stack to 
     * account for the reduced stack */
    pSAST->pCurrentFunction->t.function.stackSize -= 
	pDeclarationList->t.declarationList.stackSize;
  }

  return sgErr;

} /* sBE_GenCompoundStatement */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenExpressionProducer
 */

aErr sBE_GenExpressionProducer(aSAST* pSAST,
			       aASTNode* pNode)
{
  aErr apErr = aErrNone;

  switch (pNode->eType) {
  case tExpression:
    apErr = sBE_GenExpression(pSAST, pNode);
    break;
  case tAssignmentExpression:
    apErr = sBE_GenAssignmentExpression(pSAST, pNode);
    break;
  case tLogicalORExpression:
    apErr = sBE_GenLogicalORExpression(pSAST, pNode);
    break;
  case tLogicalANDExpression:
    apErr = sBE_GenLogicalANDExpression(pSAST, pNode);
    break;
  case tInclusiveORExpression:
  case tExclusiveORExpression:
  case tANDExpression:
  case tAdditiveExpression:
  case tMultiplicativeExpression:
    apErr = sBE_Gen2OpExpression(pSAST, pNode);
    break;
  case tEqualityExpression:
    apErr = sBE_GenEqualityExpression(pSAST, pNode);
    break;
  case tRelationalExpression:
    apErr = sBE_GenRelationalExpression(pSAST, pNode);
    break;
  case tShiftExpression:
    apErr = sBE_GenShiftExpression(pSAST, pNode);
    break;
  case tUnaryOperator:
    apErr = sBE_GenUnaryOperator(pSAST, pNode);
    break;
  case tCastExpression:
    apErr = sBE_GenCastExpression(pSAST, pNode);
    break;
  case tUnaryExpression:
    apErr = sBE_GenUnaryExpression(pSAST, pNode);
    break;
  case tPostFixExpression:
    apErr = sBE_GenPostFixExpression(pSAST, pNode);
    break;
  case tPrimaryExpression:
    apErr = sBE_GenPrimaryExpression(pSAST, pNode);
    break;
  default:
    aAssert(0); /* should we get here ? */
    break;
  } /* switch */

  return apErr;

} /* sBE_GenExpressionProducer */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenLogicalORExpression
 *
 * evaluate the left hand side
 * if it is non-zero
 *  push 1 (byte) onto the stack
 * else
 *  evaluate right hand side
 *  if it is non-zero
 *   push 1 (byte) onto the stack
 *  else
 *   push 0 onto the stack
 */

aErr sBE_GenLogicalORExpression(aSAST* pSAST,
				aASTNode* pLogicalORExpression)
{
  aErr sgErr = aErrNone;
  aASTNode* pTemp;
  tADDRESS mark1 = 0;
  tADDRESS mark2 = 0;

  aAssert(pLogicalORExpression);
  aAssert(pLogicalORExpression->eType == tLogicalORExpression);

  /* push the left side onto the stack */
  pTemp = pLogicalORExpression->pChildren;
  aAssert(pTemp);
  sgErr = sBE_GenExpressionProducer(pSAST, pTemp);

  /* pop off the left hand result */
  sgErr = sPopExpression(pSAST, pTemp);

  if (pSAST->bJustCompute) {
  
    /* test the left hand expression */
    pSAST->nCodeBytes += 3;

  } else {

    /* 3 bytes, test for non-zero */
    pSAST->code[pSAST->nCodeBytes++] = op_BRNZ;

    /* save this position, we will fill address in after the statement */
    mark1 = pSAST->nCodeBytes;
    pSAST->nCodeBytes += 2;
  }

  /* now, handle the right side expression */  
  pTemp = pTemp->pNext;
  aAssert(pTemp);  
  sgErr = sBE_GenExpressionProducer(pSAST, pTemp);

  /* pop off the right hand result */
  if (sgErr == aErrNone)
    sgErr = sPopExpression(pSAST, pTemp);

  if (pSAST->bJustCompute) {
  
    /* for the right hand expression, test, set false, goto, set true*/
    pSAST->nCodeBytes += 3 + 2 + 3 + 2;

  } else {
    tADDRESS setTrueAddr;
    
    /* 3 bytes, test for non-zero */
    pSAST->code[pSAST->nCodeBytes++] = op_BRNZ;
    /* save this position, we will fill address in after the statement */
    mark2 = pSAST->nCodeBytes;
    pSAST->nCodeBytes += 2;
    
    /* 2 bytes, set the result to false */
    pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
    pSAST->code[pSAST->nCodeBytes++] = 0;

    /* 3 bytes, jump past the set positive */
    pSAST->code[pSAST->nCodeBytes++] = op_GOTO;
    aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		      (tADDRESS)(pSAST->nCodeBytes + pSAST->nCurRoutineStart + 4));
    pSAST->nCodeBytes += sizeof(tADDRESS);

    /* patch up the branch addresses from before */
    setTrueAddr = (tADDRESS)(pSAST->nCodeBytes + pSAST->nCurRoutineStart);
    aTEA_StoreAddress(&pSAST->code[mark1], setTrueAddr);
    aTEA_StoreAddress(&pSAST->code[mark2], setTrueAddr);

    /* 2 bytes, set the result to true */
    pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
    pSAST->code[pSAST->nCodeBytes++] = 1;

    /* whether true or false, the stack has a byte pushed when done */
    pSAST->nStackOffset += 1;
  }

  return sgErr;

} /* sBE_GenLogicalORExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenLogicalANDExpression
 *
 * evaluate the left hand side
 * if it is non-zero
 *  push 1 (byte) onto the stack
 * else
 *  evaluate right hand side
 *  if it is non-zero
 *   push 1 (byte) onto the stack
 *  else
 *   push 0 onto the stack
 */

aErr sBE_GenLogicalANDExpression(aSAST* pSAST,
				 aASTNode* pLogicalANDExpression)
{
  aErr sgErr = aErrNone;
  aASTNode* pTemp;
  tADDRESS mark1 = 0;
  tADDRESS mark2;

  aAssert(pLogicalANDExpression);
  aAssert(pLogicalANDExpression->eType == tLogicalANDExpression);

  /* push the left side onto the stack */
  pTemp = pLogicalANDExpression->pChildren;
  aAssert(pTemp);
  sgErr = sBE_GenExpressionProducer(pSAST, pTemp);

  /* pop off the right hand result */
  if (sgErr == aErrNone)
    sgErr = sPopExpression(pSAST, pTemp);

  if (pSAST->bJustCompute) {
  
    /* test the left hand expression */
    pSAST->nCodeBytes += 3;

  } else {

    /* 3 bytes, test for zero */
    pSAST->code[pSAST->nCodeBytes++] = op_BRZ;
    /* save this position, we will fill address in after the statement */
    mark1 = pSAST->nCodeBytes;
    pSAST->nCodeBytes += 2;
  }

  /* now, handle the right side expression */  
  pTemp = pTemp->pNext;
  aAssert(pTemp);  
  sgErr = sBE_GenExpressionProducer(pSAST, pTemp);

  /* pop off the right hand result */
  if (sgErr == aErrNone)
    sgErr = sPopExpression(pSAST, pTemp);

  if (pSAST->bJustCompute) {
  
    /* the right hand expression, test, set true, goto, set false*/
    pSAST->nCodeBytes += 3 + 2 + 3 + 2;

  } else {
    tADDRESS setTrueAddr;
    
    /* 3 bytes, test for non-zero */
    pSAST->code[pSAST->nCodeBytes++] = op_BRZ;
    /* save this position, we will fill address in after the statement */
    mark2 = pSAST->nCodeBytes;
    pSAST->nCodeBytes += 2;
    
    /* 2 bytes, set the result to true */
    pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
    pSAST->code[pSAST->nCodeBytes++] = 1;

    /* 3 bytes, jump past the set negative */
    pSAST->code[pSAST->nCodeBytes++] = op_GOTO;
    aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		      (tADDRESS)(pSAST->nCodeBytes + 
    		      		 pSAST->nCurRoutineStart + 4));
    pSAST->nCodeBytes += sizeof(tADDRESS);

    /* patch up the branch addresses from before */
    setTrueAddr = (tADDRESS)(pSAST->nCodeBytes + pSAST->nCurRoutineStart);
    aTEA_StoreAddress(&pSAST->code[mark1], setTrueAddr);
    aTEA_StoreAddress(&pSAST->code[mark2], setTrueAddr);

    /* 2 bytes, set the result to false */
    pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
    pSAST->code[pSAST->nCodeBytes++] = 0;

    /* whether true or false, the stack has a byte pushed when done */
    pSAST->nStackOffset += 1;
  }

  return sgErr;

} /* sBE_GenLogicalANDExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_Gen2OpExpression
 */

aErr sBE_Gen2OpExpression(aSAST* pSAST, 
			  aASTNode* p2OpExpression)
{
  aErr sgErr = aErrNone;

  aAssert(p2OpExpression);

  /* see if it is a constant expression */
  if (p2OpExpression->flags & fConstant) {
  
    /* we can just treat the node as a constant */
    sgErr = sBE_GenConstant(pSAST, p2OpExpression);
  
  /* if not, perform the math */
  } else {
    aASTNode* pTemp;

    /* push the left side onto the stack */
    pTemp = p2OpExpression->pChildren;
    aAssert(pTemp);
    sgErr = sBE_GenExpressionProducer(pSAST, pTemp);

    /* now, handle the right side expression */
    pTemp = pTemp->pNext;
    aAssert(pTemp);  
    sgErr = sBE_GenExpressionProducer(pSAST, pTemp);

    if (pSAST->bJustCompute) {
  
      /* perform the operation */
      pSAST->nCodeBytes += 1;

      /* special case that converts the short result to char */
      if ((p2OpExpression->flags & f2OpMULT) 
          && (pTemp->flags & fByte))
        pSAST->nCodeBytes += 1;

    } else {

      /* 1 byte, for opcode */
      if (p2OpExpression->flags & f2OpOR) {
        if (pTemp->flags & fByte)
          pSAST->code[pSAST->nCodeBytes++] = op_ORB;
        else
          pSAST->code[pSAST->nCodeBytes++] = op_ORS;
      } else if (p2OpExpression->flags & f2OpXOR) {
        if (pTemp->flags & fByte)
          pSAST->code[pSAST->nCodeBytes++] = op_XORB;
        else
          pSAST->code[pSAST->nCodeBytes++] = op_XORS;
      } else if (p2OpExpression->flags & f2OpAND) {
        if (pTemp->flags & fByte)
          pSAST->code[pSAST->nCodeBytes++] = op_ANDB;
        else
          pSAST->code[pSAST->nCodeBytes++] = op_ANDS;
      } else if (p2OpExpression->flags & f2OpPLUS) {
        if (pTemp->flags & fByte)
          pSAST->code[pSAST->nCodeBytes++] = op_ADDB;
        else
          pSAST->code[pSAST->nCodeBytes++] = op_ADDS;
      } else if (p2OpExpression->flags & f2OpMINUS) {
        if (pTemp->flags & fByte)
          pSAST->code[pSAST->nCodeBytes++] = op_SUBB;
        else
          pSAST->code[pSAST->nCodeBytes++] = op_SUBS;
      } else if (p2OpExpression->flags & f2OpMULT) {
        if (pTemp->flags & fByte) {
          pSAST->code[pSAST->nCodeBytes++] = op_MULTB;
          /* special case that converts the short result to char */
          pSAST->code[pSAST->nCodeBytes++] = op_CONVSB;
        } else
          pSAST->code[pSAST->nCodeBytes++] = op_MULTS;
      } else if (p2OpExpression->flags & f2OpDIV) {
        if (pTemp->flags & fByte)
          pSAST->code[pSAST->nCodeBytes++] = op_DIVB;
        else
          pSAST->code[pSAST->nCodeBytes++] = op_DIVS;
      } else if (p2OpExpression->flags & f2OpMOD) {
        if (pTemp->flags & fByte)
          pSAST->code[pSAST->nCodeBytes++] = op_MODB;
        else
          pSAST->code[pSAST->nCodeBytes++] = op_MODS;
      }

      /* account for the stack reduction created by this op */    
      pSAST->nStackOffset -= sTypeSizeFromFlags(pTemp);
    }
  }

  return sgErr;

} /* sBE_Gen2OpExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenEqualityExpression
 *
 * we want something like
 *
 * evaluate a, evaluate b
 * sub a from b		    1 byte
 * remove result	    2 bytes
 * if b op a branch to yes  3 bytes
 *   pushlb 0               2 bytes
 *   branch                 3 bytes
 * yes:
 *   pushlb 1               2 bytes
 */

aErr sBE_GenEqualityExpression(aSAST* pSAST,
			       aASTNode* pEqualityExpression)
{
  aErr sgErr = aErrNone;
  aASTNode* pTemp;

  aAssert(pEqualityExpression);
  aAssert(pEqualityExpression->eType == tEqualityExpression);

  /* push the left side onto the stack */
  pTemp = pEqualityExpression->pChildren;
  aAssert(pTemp);
  sgErr = sBE_GenExpressionProducer(pSAST, pTemp);

  /* now, handle the right side expression */  
  pTemp = pTemp->pNext;
  aAssert(pTemp);  
  sgErr = sBE_GenExpressionProducer(pSAST, pTemp);

  if (pSAST->bJustCompute) {

    /* popn the left hand expression and test */
    pSAST->nCodeBytes += 1 + 1 + 3 + 2 + 3 + 2;

  } else {

    /* first, the subtraction and popn */
    if (pTemp->flags & fByte) {
      pSAST->code[pSAST->nCodeBytes++] = op_SUBB;
      pSAST->code[pSAST->nCodeBytes++] = op_POPB;
      pSAST->nStackOffset -= 2;
    } else if (pTemp->flags & fShort) {
      pSAST->code[pSAST->nCodeBytes++] = op_SUBS;
      pSAST->code[pSAST->nCodeBytes++] = op_POPS;
      pSAST->nStackOffset -= 4;
    }

    /* then the test and branch to success */
    if (pEqualityExpression->flags & fEquality) {
      pSAST->code[pSAST->nCodeBytes++] = op_BRZ;
    } else if (pEqualityExpression->flags & fNotEquality) {
      pSAST->code[pSAST->nCodeBytes++] = op_BRNZ;
    }
    aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		      (tADDRESS)(pSAST->nCodeBytes 
    		                 + pSAST->nCurRoutineStart 
    		                 + 7));
    pSAST->nCodeBytes += sizeof(tADDRESS);
    
    /* then the no */
    pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
    pSAST->code[pSAST->nCodeBytes++] = 0;

    /* and the branch over the yes */
    pSAST->code[pSAST->nCodeBytes++] = op_GOTO;
    aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		      (tADDRESS)(pSAST->nCodeBytes 
    		                 + pSAST->nCurRoutineStart 
    		                 + 4));
    pSAST->nCodeBytes += sizeof(tADDRESS);
    
    /* finally, the yes */
    pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
    pSAST->code[pSAST->nCodeBytes++] = 1;
    
    /* account for the result */
    pSAST->nStackOffset += 1;
  }

  return sgErr;

} /* sBE_GenEqualityExpression */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenRelationalExpression
 */

aErr sBE_GenRelationalExpression (
  aSAST* pSAST,
  aASTNode* pRelationalExpression
)
{
  aErr sgErr = aErrNone;
  aASTNode* pTemp;
  aBool bUnsignedChar = aFalse;
  aBool bUnsignedInt = aFalse;
  aBool bSignedChar = aFalse;
  aBool bSignedInt = aFalse;

  aAssert(pRelationalExpression);
  aAssert(pRelationalExpression->eType == tRelationalExpression);

  /* push the left side onto the stack */
  pTemp = pRelationalExpression->pChildren;
  aAssert(pTemp);
  sgErr = sBE_GenExpressionProducer(pSAST, pTemp);
  
  /* now, handle the right side expression */  
  pTemp = pTemp->pNext;
  aAssert(pTemp);  
  sgErr = sBE_GenExpressionProducer(pSAST, pTemp);

  /* assign handy type flags */
  if (pTemp->flags & fUnsigned) {
    if (pTemp->flags & fShort) {
      bUnsignedInt = aTrue;
    } else {
      bUnsignedChar = aTrue;
    }
  } else {
    if (pTemp->flags & fShort) {
      bSignedInt = aTrue;
    } else {
      bSignedChar = aTrue;
    }
  }

  if (pSAST->bJustCompute) {

    /* popn the left hand expression and test */

    if (!(pTemp->flags & fUnsigned)
        && (pTemp->flags & fByte)) {

      pSAST->nCodeBytes += 2  /* pushsb 2 */
			 + 1  /* convbs */
			 + 2  /* pushsb 3 */
			 + 1  /* convbs */
			 + 1  /* subs */
			 + 2  /* popn 4 */
			 + 3  /* branch based on operation */
			 + 2  /* pushlb, false */
			 + 3  /* goto end */
			 + 2; /* pushlb, true */
    
    } else if (!(pTemp->flags & fUnsigned)
	       && (pTemp->flags & fShort)) {

      pSAST->nCodeBytes += 2  /* pushss 4 */
			 + 3  /* pushls 0x8000 */
			 + 1  /* adds */
			 + 2  /* pushsb 4 */
			 + 3  /* pushls 0x8000 */
			 + 1  /* adds */
			 + 1  /* subs */
			 + 2  /* popn 6 */
			 + 3  /* branch based on operation */
			 + 2  /* pushlb, false */
			 + 3  /* goto end */
			 + 2; /* pushlb, true */

    } else {

      pSAST->nCodeBytes += 1  /* subb */
    		         + 1  /* popb or pops */
    		         + 3  /* branch based on operation */
    		         + 2  /* pushlb, false */
    		         + 3  /* goto end */
    		         + 2; /* pushlb, true */
    }

    /* signed int uses unsigned math after a range shift */
    /* so its extra branch rules are same as unsigned char */
    
    if ((!bSignedChar)
        && !(pRelationalExpression->flags & fRelateEQ)
    	&& (pRelationalExpression->flags & fRelateGT))
      pSAST->nCodeBytes += 3;  /* > extra branch */

    if ((!bSignedChar)
        && (pRelationalExpression->flags & fRelateEQ)
    	&& (pRelationalExpression->flags & fRelateLT))
      pSAST->nCodeBytes += 3;  /* <= extra branch */

    /* signed char <= or >= */
    if (bSignedChar
        && (pRelationalExpression->flags & fRelateEQ)
    	&& (pRelationalExpression->flags & (fRelateGT | fRelateLT)))
      pSAST->nCodeBytes += 3;  /* extra branch */

  } else {

    /* first do the subtraction and pop of the result */
    if (pTemp->flags & fByte) {

      /* signed char op requires conversion to short */
      /* (this is more efficient than a range shift) */
      if (!(pTemp->flags & fUnsigned)) {
	pSAST->code[pSAST->nCodeBytes++] = op_PUSHSB;
	pSAST->code[pSAST->nCodeBytes++] = 2;
	pSAST->code[pSAST->nCodeBytes++] = op_CONVBS;
	pSAST->code[pSAST->nCodeBytes++] = op_PUSHSB;
	pSAST->code[pSAST->nCodeBytes++] = 3;
	pSAST->code[pSAST->nCodeBytes++] = op_CONVBS;
	pSAST->code[pSAST->nCodeBytes++] = op_SUBS;
	pSAST->code[pSAST->nCodeBytes++] = op_POPN;
	pSAST->code[pSAST->nCodeBytes++] = 4;
      } else {
	pSAST->code[pSAST->nCodeBytes++] = op_SUBB;
	pSAST->code[pSAST->nCodeBytes++] = op_POPB;
      }
      pSAST->nStackOffset -= 2;
      
    } else if (pTemp->flags & fShort) {

      /* signed int op requires range shift */
      /* (this converts it to an unsigned comparison) */
      if (!(pTemp->flags & fUnsigned)) {
	pSAST->code[pSAST->nCodeBytes++] = op_PUSHSS;
	pSAST->code[pSAST->nCodeBytes++] = 4;
	pSAST->code[pSAST->nCodeBytes++] = op_PUSHLS;
	pSAST->code[pSAST->nCodeBytes++] = (char)0x80;
	pSAST->code[pSAST->nCodeBytes++] = 0;
	pSAST->code[pSAST->nCodeBytes++] = op_ADDS;
	pSAST->code[pSAST->nCodeBytes++] = op_PUSHSS;
	pSAST->code[pSAST->nCodeBytes++] = 4;
	pSAST->code[pSAST->nCodeBytes++] = op_PUSHLS;
	pSAST->code[pSAST->nCodeBytes++] = (char)0x80;
	pSAST->code[pSAST->nCodeBytes++] = 0;
	pSAST->code[pSAST->nCodeBytes++] = op_ADDS;
	pSAST->code[pSAST->nCodeBytes++] = op_SUBS;
	pSAST->code[pSAST->nCodeBytes++] = op_POPN;
	pSAST->code[pSAST->nCodeBytes++] = 6;
      } else {
	pSAST->code[pSAST->nCodeBytes++] = op_SUBS;
	pSAST->code[pSAST->nCodeBytes++] = op_POPS;
      }
      pSAST->nStackOffset -= 4;
    }

    if (pRelationalExpression->flags & fRelateEQ) {
      if (pRelationalExpression->flags & fRelateLT) {
        /* unsigned, signed int <= */
        if (!bSignedChar) {
          pSAST->code[pSAST->nCodeBytes++] = op_BRZ;
          aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		            (tADDRESS)(pSAST->nCodeBytes 
    		                 + pSAST->nCurRoutineStart 
    		                 + 10));
          pSAST->nCodeBytes += sizeof(tADDRESS);
          pSAST->code[pSAST->nCodeBytes++] = op_BRNC;
        }
        /* signed char <= */
        else {
          pSAST->code[pSAST->nCodeBytes++] = op_BRZ;
          aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		            (tADDRESS)(pSAST->nCodeBytes 
    		                 + pSAST->nCurRoutineStart 
    		                 + 10));
          pSAST->nCodeBytes += sizeof(tADDRESS);
          pSAST->code[pSAST->nCodeBytes++] = op_BRNEG;
        }

      } else if (pRelationalExpression->flags & fRelateGT) {
        /* unsigned, signed int >= */
        if (!bSignedChar)
          pSAST->code[pSAST->nCodeBytes++] = op_BRC;
        /* signed char >= */ 
        else {
          pSAST->code[pSAST->nCodeBytes++] = op_BRZ;
          aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		            (tADDRESS)(pSAST->nCodeBytes 
    		                 + pSAST->nCurRoutineStart 
    		                 + 10));
          pSAST->nCodeBytes += sizeof(tADDRESS);
          pSAST->code[pSAST->nCodeBytes++] = op_BRPOS;
        }
      }
    } else {
      if (pRelationalExpression->flags & fRelateLT) {
        /* unsigned, signed int < */ 
        if (!bSignedChar)
          pSAST->code[pSAST->nCodeBytes++] = op_BRNC;
        /* signed char < */ 
        else
          pSAST->code[pSAST->nCodeBytes++] = op_BRNEG;
      } else if (pRelationalExpression->flags & fRelateGT) {
        /* unsigned, signed int > */
        if (!bSignedChar) {
          pSAST->code[pSAST->nCodeBytes++] = op_BRNC;
          aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		            (tADDRESS)(pSAST->nCodeBytes 
    		                 + pSAST->nCurRoutineStart 
    		                 + 5));
          pSAST->nCodeBytes += sizeof(tADDRESS);
          pSAST->code[pSAST->nCodeBytes++] = op_BRNZ;
        }
        /* signed char > */
        else
          pSAST->code[pSAST->nCodeBytes++] = op_BRPOS;
      }
    }
    
    /* set the branch too address */  
    aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		      (tADDRESS)(pSAST->nCodeBytes 
    		                 + pSAST->nCurRoutineStart 
    		                 + 7));
    pSAST->nCodeBytes += sizeof(tADDRESS);
 
    /* set up the no (2 bytes) */
    pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
    pSAST->code[pSAST->nCodeBytes++] = 0;

    /* and the branch over the yes (3 bytes) */
    pSAST->code[pSAST->nCodeBytes++] = op_GOTO;
    aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		      (tADDRESS)(pSAST->nCodeBytes 
    		                 + pSAST->nCurRoutineStart 
    		                 + 4));
    pSAST->nCodeBytes += sizeof(tADDRESS);

    /* finally, the yes (2 bytes) */
    pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
    pSAST->code[pSAST->nCodeBytes++] = 1;

    /* account for the result */
    pSAST->nStackOffset += 1;

  } /* if not just compute */

  return sgErr;

} /* sBE_GenRelationalExpression */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenShiftExpression
 */

aErr sBE_GenShiftExpression(aSAST* pSAST,
			    aASTNode* pShiftExpression)
{
  aErr sgErr = aErrNone;

  aAssert(pShiftExpression);
  aAssert(pShiftExpression->eType == tShiftExpression);

  /* constant expressions need not be computed in the vm */
  if (pShiftExpression->flags & fConstant) {

    /* we can just treat the node as a constant */
    sgErr = sBE_GenConstant(pSAST, pShiftExpression);

  } else {
    aASTNode* pLeft;
    aASTNode* pRight;
    
    pLeft = pShiftExpression->pChildren;
    aAssert(pLeft);
    pRight = pLeft->pNext;
    aAssert(pRight);
 
    /* push on the left and right values onto the 
     * stack */
    sgErr = sBE_GenExpressionProducer(pSAST, pLeft);
    if (sgErr == aErrNone)
      sgErr = sBE_GenExpressionProducer(pSAST, pRight);
 
 
    /* if the right side is a short, we need to 
     * convert it.  We need to catch the case where 
     * the short was huge.  We just do the maximum 
     * relevant shift in this case. */
    if (sgErr == aErrNone) {
      if (pRight->flags & fShort) {
        if (pSAST->bJustCompute == aTrue) {
          pSAST->nCodeBytes += 1; /* op_CONVSB */
          pSAST->nCodeBytes += 3; /* op_BRC */
          pSAST->nCodeBytes += 1; /* op_POPB */
          pSAST->nCodeBytes += 2; /* op_PUSHLB */
        } else {
          pSAST->code[pSAST->nCodeBytes++] = op_CONVSB;
          pSAST->nStackOffset--;
          pSAST->code[pSAST->nCodeBytes++] = op_BRNC;
          aTEA_StoreAddress(&pSAST->code[pSAST->nCodeBytes], 
    		          (tADDRESS)(pSAST->nCodeBytes 
    		                 + pSAST->nCurRoutineStart 
    		                 + 5));
          pSAST->nCodeBytes += sizeof(tADDRESS);
          pSAST->code[pSAST->nCodeBytes++] = op_POPB;
          pSAST->code[pSAST->nCodeBytes++] = op_PUSHLB;
          if (pLeft->flags & fByte)
            pSAST->code[pSAST->nCodeBytes++] = 8;
          else
            pSAST->code[pSAST->nCodeBytes++] = 16;
        }
      }
    }

    /* do the actual shift */
    if (sgErr == aErrNone) {
      if (pSAST->bJustCompute == aTrue)
        pSAST->nCodeBytes += 1; /* op_RLX, op RRX */
      else {
        if (pShiftExpression->flags & fByte) {
          if (pShiftExpression->flags & fShiftLeft)
            pSAST->code[pSAST->nCodeBytes++] = op_RLB;
          else
            pSAST->code[pSAST->nCodeBytes++] = op_RRB;
        } else if (pShiftExpression->flags & fShort) {
        /* fix by MRW 10-04-01 */
        /*
            pSAST->code[pSAST->nCodeBytes++] = op_RLS;
        } else {
            pSAST->code[pSAST->nCodeBytes++] = op_RRS;
        */
          if (pShiftExpression->flags & fShiftLeft)
            pSAST->code[pSAST->nCodeBytes++] = op_RLS;
          else
            pSAST->code[pSAST->nCodeBytes++] = op_RRS;
        }
        /* the shift amount comes off the stack */
        pSAST->nStackOffset--;
      }
    }
  }
  
  return sgErr;

} /* sBE_GenShiftExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBE_GenCastExpression
 */

aErr sBE_GenCastExpression(aSAST* pSAST,
			   aASTNode* pCastExpression)
{
  aErr sgErr = aErrNone;

  aAssert(pCastExpression);
  aAssert(pCastExpression->eType == tCastExpression);

  /* constant expressions need not be computed in the vm */
  if (pCastExpression->flags & fConstant) {

    /* we can just treat the node as a constant */
    sgErr = sBE_GenConstant(pSAST, pCastExpression);

  } else {
    aASTNode* pType = pCastExpression->pChildren;
    aASTNode* pChild = pType->pNext;
 
    /* first, we always need to have the expression on the stack */
    sgErr = sBE_GenExpressionProducer(pSAST, pChild);

    /* now, only worry about it if the types differ */
    if ((pType->flags & fTypeMask) 
    	!= (pChild->flags & fTypeMask)) {
      if (pSAST->bJustCompute) {
        pSAST->nCodeBytes++; /* for conv */
      } else {
        if ((pType->flags & fByte)
            && (pChild->flags & fShort)) {
          pSAST->code[pSAST->nCodeBytes++] = op_CONVSB;
          pSAST->nStackOffset--;
        } else if ((pType->flags & fShort)
                   && (pChild->flags & fByte)) {
          pSAST->code[pSAST->nCodeBytes++] = op_CONVBS;
          pSAST->nStackOffset++;
        }
      }
    }
  }

  return sgErr;

} /* sBE_GenCastExpression */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepGenInternal_Create
 */

aErr aSteepGenInternal_Create(aSteepGen** ppSteepGen)
{
  aErr soErr = aErrNone;
  aSteepGen* pSteepGen = NULL;

  if (ppSteepGen == NULL)
    soErr = aErrParam;

  if (soErr == aErrNone) {
    pSteepGen = (aSteepGen*)aMemAlloc(sizeof(aSteepGen));
    if (pSteepGen == NULL) {
      soErr = aErrParam;
    } else {
      aBZero(pSteepGen, sizeof(aSteepGen));
      pSteepGen->check = aSTEEPGENCHECK;
      if (soErr != aErrNone)
        aSteepGenInternal_Destroy(pSteepGen);
    }
  }
  
  if (soErr == aErrNone)
    *ppSteepGen = pSteepGen;
  
  return soErr;

} /* aSteepGenInternal_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepGenInternal_Generate
 */

aErr aSteepGenInternal_Generate(aSteepGen* pSteepGen,
				aSAST* pSAST,
			        aStreamRef result)
{
  aErr sgErr = aErrNone;
  aMemSize nMemHighWater = 0;
 
  aAssert(pSAST);
  aAssert(pSAST->pTree);
  aAssert(result);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * generate an error summary if there were compile errors 
   */
  if ((sgErr == aErrNone)
      && (pSAST->nErrors != 0)) {
    char num[16];
    char line[100];
    aStringFromInt(num, pSAST->nErrors);
    aStringCopy(line, num);
    aStringCat(line, aTE_TOTAL_ERRORS);
    sgErr = pSAST->outputErrProc(pSAST, NULL, line);    
  } else {
    pSAST->outputStream = result;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * write out the signature and version
   */
  if ((sgErr == aErrNone)
      && (pSAST->nErrors == 0)) {
    char v;
    if (!aStream_Write(aStreamLibRef(result), result, 
    		       aTEA_2BYTE_SIGNATURE, 2, &sgErr)) {
      v = aTEA_VERSION_MAJOR;
      if (!aStream_Write(aStreamLibRef(result), result, 
      			 &v, 1, &sgErr)) {
        v = aTEA_VERSION_MINOR;
        aStream_Write(aStreamLibRef(result), result, 
        	      &v, 1, &sgErr);
      }
    }
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * now compute the main routine size for the header params
   */
  if ((sgErr == aErrNone)
      && (pSAST->pMainRoutine != NULL)) {
    char code[3];
    tBYTE nReturnBytes;
    tBYTE nParamBytes;
    tADDRESS nPreambleSize = 0;
    aASTNode* pExternalDeclaration;
    unsigned char nGlobalStackSize = 0;
    tADDRESS nGlobalCodeBytes = 0;

    /* establish the calling parameter and return value sizes */
    pSAST->bJustCompute = aTrue;
    sgErr = sBE_GetCallSizes(pSAST->pMainRoutine, 
    			     &nReturnBytes, &nParamBytes);

    /* write out the return value size for the entry routine */
    if (sgErr == aErrNone) 
      aStream_Write(aStreamLibRef(pSAST->outputStream),
  		    pSAST->outputStream, (void*)&nReturnBytes, 1, &sgErr);

    /* write out the parameter size for the entry routine */
    if (sgErr == aErrNone) 
      aStream_Write(aStreamLibRef(pSAST->outputStream),
  		    pSAST->outputStream, (void*)&nParamBytes, 1, &sgErr);

    /* compute the preamble size which includes global parameter
     * initialization */
    if (sgErr == aErrNone) {

      /* show the return value and parameters on the stack first */
      pSAST->nStackOffset = (tSTACK)(nReturnBytes + nParamBytes);

      /* step through the external declarations seeking globals */
      pExternalDeclaration = pSAST->pTree->pChildren;
      while (pExternalDeclaration && (sgErr == aErrNone)) {
        aASTNode* pDeclaration = pExternalDeclaration->pChildren;
        aAssert(pExternalDeclaration->eType == tExternalDeclaration);
        if (pDeclaration && (pDeclaration->eType == tDeclaration))
          sgErr = sBE_GenDeclaration(pSAST, pDeclaration);
        pExternalDeclaration = pExternalDeclaration->pNext;
      }

      /* pSAST->nStackOffset now contains total stack size of 
       *  return value and global variables combined.
       * pSAST->nCodeBytes contains initialization code size 
       *  for globals */
      nGlobalCodeBytes = pSAST->nCodeBytes;

      /* save the global stack size and add it to the total 
       * routine stack size when we are done computing 
       * the routine stack information since the globals 
       * sit on top of the routines calling stack values 
       * (return value and parameters) */
      nGlobalStackSize = (unsigned char)(pSAST->nStackOffset
      					 - (nReturnBytes + nParamBytes));
      if ((nGlobalStackSize > 0) || (nParamBytes > 0))
        nPreambleSize = 6;
      else
        nPreambleSize = 4;
      nPreambleSize += nGlobalCodeBytes;
      pSAST->nCurRoutineStart = nPreambleSize;
      nMemHighWater = nGlobalCodeBytes;
    }

    /* main is always the first routine called */
    /* must keep record of global stack size */
    if (sgErr == aErrNone) {
      pSAST->nGlobalStackSize = nGlobalStackSize;
      sgErr = sBE_GenFunctionDefinition(pSAST, pSAST->pMainRoutine);
    }
 
    /* add in the main routine size */
    if (sgErr == aErrNone) {
      if (pSAST->pMainRoutine->nCodePosition > nMemHighWater)
        nMemHighWater = pSAST->pMainRoutine->nCodePosition;
      pSAST->nCurRoutineStart += pSAST->pMainRoutine->nCodePosition;
    }

    /* compute all the other routines */
    if (sgErr == aErrNone) {
      aASTNode* pDeclaration = pSAST->pTree->pChildren;
      /* step through the external declarations */
      while (pDeclaration && (sgErr == aErrNone)) {
        aASTNode* pFunction = pDeclaration->pChildren;
        if (pFunction
            && (pFunction->eType == tFunctionDefinition)
            && (pFunction != pSAST->pMainRoutine)
            && (pFunction->flags & fFunctionUsed)) {
          sgErr = sBE_GenFunctionDefinition(pSAST, pFunction);
          pSAST->nCurRoutineStart += pFunction->nCodePosition;
          if (pFunction->nCodePosition > nMemHighWater)
            nMemHighWater = pFunction->nCodePosition;
        }
        pDeclaration = pDeclaration->pNext;
      }
    }

    /* write the entry preamble, file looks like:
     *   byte	addr   value
     * ------    header   -------
     *    0	 	'a'
     *    1	 	'T'
     *    2	 	version high
     *    3	 	version low
     *    5	 	return value size
     *    6	 	parameter data size
     * ------  code start -------
     *    7	 0000	op_CALL       (loader has return value 
     *    8	 0001	0	       space and input data
     *    9	 0002	6	       in place already)
     *   10	 0003	op_POPN       (this is not in place for
     *   11	 0004	param size     void return value)
     *   12	 0005	op_EXIT
     *   13	 0006	entry routine
     *    :
     *    :	 routine
     *    :
     *    n	 n-7	op_RETURN
     */

    /* build a memory block for generating code */
    if (sgErr == aErrNone) {
      aAssert(pSAST->code == NULL);
      pSAST->code = aMemAlloc(nMemHighWater);
      aAssert(pSAST->code);
    }

    /* dump the global initialization code */
    if ((sgErr == aErrNone)
        && (nGlobalCodeBytes > 0)) {

      /* set up the SAST block for code generation */
      pSAST->nCodeBytes = 0;
      pSAST->pCurrentFunction = NULL;
      pSAST->bJustCompute = aFalse;
    
      /* the stack starts with just the return value space */
      pSAST->nStackOffset = nReturnBytes;
      pExternalDeclaration = pSAST->pTree->pChildren;
      /* step through the external declarations seeking globals */
      while (pExternalDeclaration && (sgErr == aErrNone)) {
        aASTNode* pDeclaration = pExternalDeclaration->pChildren;
        aAssert(pExternalDeclaration->eType == tExternalDeclaration);
        if (pDeclaration && (pDeclaration->eType == tDeclaration))
          sgErr = sBE_GenDeclaration(pSAST, pDeclaration);
        pExternalDeclaration = pExternalDeclaration->pNext;
      }
   
      aAssert(pSAST->nCodeBytes == nGlobalCodeBytes);
      
      if (sgErr == aErrNone)
        aStream_Write(pSAST->ioRef, pSAST->outputStream,
  		      pSAST->code, nGlobalCodeBytes, &sgErr);
    }

    if (sgErr == aErrNone) {

      /* store the current instruction that starts the 
         routine code (for routine-relative jumps) */
      pSAST->nCurRoutineStart = nPreambleSize;

      /* store the call to the main routine */
      code[0] = op_CALL;
      aTEA_StoreAddress(&code[1], nPreambleSize);
      aStream_Write(aStreamLibRef(pSAST->outputStream),
  		    pSAST->outputStream, code, 3, &sgErr);
    }

    /* only pop bytes if there were parameters or globals */
    if ((sgErr == aErrNone)
        && ((nGlobalStackSize > 0) || (nParamBytes > 0))) {
      code[0] = op_POPN;
      code[1] = (char)(nParamBytes + nGlobalStackSize);
      aStream_Write(aStreamLibRef(pSAST->outputStream),
  		    pSAST->outputStream, code, 2, &sgErr);
    }

    /* always write the exit */
    if (sgErr == aErrNone) {
      code[0] = op_EXIT;
      aStream_Write(aStreamLibRef(pSAST->outputStream),
  		    pSAST->outputStream, code, 1, &sgErr);
    }

    /* now dump the routines */
    if (sgErr == aErrNone) {
      sgErr = sOutputRoutine(pSAST, pSAST->pMainRoutine);
      pSAST->nCurRoutineStart += pSAST->pMainRoutine->nCodePosition;
    }
    if (sgErr == aErrNone) {
      aASTNode* pDeclaration = pSAST->pTree->pChildren;
      /* step through the external declarations */
      while (pDeclaration && (sgErr == aErrNone)) {
        aASTNode* pFunction = pDeclaration->pChildren;
        if (pFunction
            && (pFunction->eType == tFunctionDefinition)
            && (pFunction != pSAST->pMainRoutine)
            && (pFunction->flags & fFunctionUsed)) {
          sgErr = sOutputRoutine(pSAST, pFunction); 
          pSAST->nCurRoutineStart += pFunction->nCodePosition;
	}
        pDeclaration = pDeclaration->pNext;
      }
    }

    /* free up the code now the code preparation block when done */
    if (pSAST->code) {
      aMemFree(pSAST->code);
      pSAST->code = NULL;
    }
  }

  return sgErr;

} /* aSteepGenInternal_Generate */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepGenInternal_Destroy
 */

aErr aSteepGenInternal_Destroy(aSteepGen* pSteepGen)
{
  aErr sgErr = aErrNone;

  aVALIDSTEEPGEN(pSteepGen);

  if (sgErr == aErrNone) {
    pSteepGen->check = 0;
    aMemFree((aMemPtr)pSteepGen);
  }
  
  return sgErr;

} /* aSteepGenInternal_Destroy */

