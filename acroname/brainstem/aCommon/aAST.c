/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aAST.c 						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of an AST (annotated syntax tree) for   */
/*		use in compilers. 		                   */
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

#include "aAST.h"
#include "aAST_Text.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sASTNodeSymDelete(void* pData, void* ref);
static aToken* sASTGetLeftMostToken(const aASTNode* pNode);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aASTNode_Create
 */

aErr aASTNode_Create(aAST* pAST,
		     aASTType eType,
		     aASTNode** ppNode)
{
  aErr astErr = aErrNone;
#ifdef aASTOUTPUT
  static unsigned short num = 0;
#endif
  aMemPool_Alloc(pAST->ioRef, pAST->ASTNodePool, 
  		 (void**)ppNode, &astErr);

  if (astErr == aErrNone) {
    aBZero(*ppNode, sizeof(aASTNode));
    (*ppNode)->eType = eType;
#ifdef aASTOUTPUT
    (*ppNode)->count = num++;
#endif
  }

  return astErr;
  
} /* aASTNode_Create */


#ifdef aASTOUTPUT

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aASTNode_Output
 */

aErr aASTNode_Output(aASTNode* pASTNode, 
		     const int depth,
		     aStreamRef out)
{
  aErr astErr = aErrNone;
  unsigned long indentChars;
  char indent[201];
  char line[200];
  char num[10];

  /* check for infinite recursion */
  if (depth >= 50) {
    return astErr;
  }

  /* output the header */
  if (astErr == aErrNone) {

    /* build the indent up */
    for (indentChars = 0; 
	     indentChars < (unsigned long)depth * 2; 
		 indentChars++) {
      indent[indentChars] = ' ';
    }
    indent[indentChars] = 0;

    if (!aStream_Write(aStreamLibRef(out), out, indent, indentChars, &astErr))
      aStream_WriteLine(aStreamLibRef(out), out, "--- AST Node ---", &astErr);
  }

  /* output the node type */
  if (astErr == aErrNone) {
    if (!aStream_Write(aStreamLibRef(out), out, indent, indentChars, &astErr)) {
      aStringCopy(line, "type: ");
      switch (pASTNode->eType) {
      case tTranslationUnit:
        aStringCat(line, "translation-unit");		break;
      case tExternalDeclaration:
        aStringCat(line, "external-declaration");      	break;
      case tFunctionDefinition:
        aStringCat(line, "function-definition");      	break;
      case tDeclarationSpecifier:
        aStringCat(line, "declaration-specifier");	break;
      case tTypeSpecifier:
      	aStringCat(line, "type-specifier");		break;
      case tInitDeclaratorList:
      	aStringCat(line, "init-declarator-list");	break;
      case tInitDeclarator:
      	aStringCat(line, "init-declarator");		break;
      case tDeclarator:
      	aStringCat(line, "declarator");			break;
      case tParameterList:
      	aStringCat(line, "parameter-list");		break;
      case tParameterDeclaration:
      	aStringCat(line, "parameter-declaration");	break;
      case tCompoundStatement:
        aStringCat(line, "compound-statement");		break;
      case tDeclarationList:
      	aStringCat(line, "declaration-list");		break;
      case tDeclaration:
      	aStringCat(line, "declaration");		break;
      case tStatementList:
      	aStringCat(line, "statement-list");		break;
      case tStatement:
      	aStringCat(line, "statement");			break;
      case tAsmStatement:
      	aStringCat(line, "asm-statement");		break;
      case tJumpStatement:
      	aStringCat(line, "jump-statement");		break;
      case tAsmList:
      	aStringCat(line, "asm-list");			break;
      case tOpcode:
      	aStringCat(line, "opcode");			break;
      case tExpression:
      	aStringCat(line, "expression");			break;
      case tConstant:
      	aStringCat(line, "constant");			break;
      case tString:
      	aStringCat(line, "string");			break;
      case tIdentifier:
      	aStringCat(line, "identifier");			break;
      case tLabeledStatement:
      	aStringCat(line, "labeled-statement");		break;
      case tExpressionStatement:
      	aStringCat(line, "expression-statement");	break;
      case tAssignmentExpression:
      	aStringCat(line, "assignment-expression");	break;
      case tUnaryExpression:
      	aStringCat(line, "unary-expression");		break;
      case tAssignmentOperator:
      	aStringCat(line, "assignment-operator");	break;
      case tPostFixExpression:
      	aStringCat(line, "postfix-expression");		break;
      case tArgumentExpressionList:
      	aStringCat(line, "argument-expression-list");	break;
      case tPrimaryExpression:
      	aStringCat(line, "primary-expression");		break;
      case tSelectionStatement:
      	aStringCat(line, "selection-statement");	break;
      case tInitializer:
      	aStringCat(line, "initializer");		break;
      case tUnaryOperator:
      	aStringCat(line, "unary-operator");		break;
      case tCastExpression:
      	aStringCat(line, "cast-expression");		break;
      case tConditionalExpression:
      	aStringCat(line, "conditional-expression");	break;
      case tLogicalORExpression:
      	aStringCat(line, "logical-OR-expression");	break;
      case tLogicalANDExpression:
      	aStringCat(line, "logical-AND-expression");	break;
      case tInclusiveORExpression:
      	aStringCat(line, "inclusive-OR-expression");	break;
      case tExclusiveORExpression:
      	aStringCat(line, "exclusive-OR-expression");	break;
      case tANDExpression:
      	aStringCat(line, "AND-expression");		break;
      case tEqualityExpression:
      	aStringCat(line, "equality-expression");	break;
      case tRelationalExpression:
      	aStringCat(line, "relational-expression");	break;
      case tShiftExpression:
      	aStringCat(line, "shift-expression");		break;
      case tAdditiveExpression:
      	aStringCat(line, "additive-expression");	break;
      case tMultiplicativeExpression:
      	aStringCat(line, "multiplicative-expression");	break;
      case tLoopStatement:
      	aStringCat(line, "loop-statement");		break;
      case tReflexUnit:
      	aStringCat(line, "reflex-unit");		break;
      case tReflexDeclaration:
      	aStringCat(line, "reflex-declaration");		break;
      case tModuleDeclaration:
      	aStringCat(line, "module-declaration");		break;
      case tMessageDeclaration:
      	aStringCat(line, "message-declaration");	break;
      case tVectorDeclaration:
      	aStringCat(line, "vector-declaration");		break;
      case tInlineDeclaration:
      	aStringCat(line, "inline-declaration");		break;
      case tModuleSpecifier:
      	aStringCat(line, "module-specifier");		break;
      case tMessageSpecifier:
      	aStringCat(line, "message-specifier");		break;
      case tVectorSpecifier:
      	aStringCat(line, "vector-specifier");		break;
      case tPacketData:
      	aStringCat(line, "packet-data");		break;
      case tMessageReferenceList:
      	aStringCat(line, "message-reference-list");	break;
      case tMessageReference:
      	aStringCat(line, "message-reference");		break;
      case tLModifyMessageReference:
      	aStringCat(line, "lmodify-message-reference");	break;
      case tRModifyMessageReference:
      	aStringCat(line, "rmodify-message-reference");	break;
      case tMessageModifyReference:
      	aStringCat(line, "message-modify-reference");	break;
      case tConstantValue:
      	aStringCat(line, "constant-value");		break;
      default:
        aStringCat(line, "unknown");
        astErr = aErrUnknown;
        break;
      } /* eType switch */
      aStream_WriteLine(aStreamLibRef(out), out, line, &astErr);
    }
  }
  
  /* output the node flags */
  if ((astErr == aErrNone)
      && !aStream_Write(aStreamLibRef(out), out, indent, indentChars, &astErr)) {
    aBool sep = aFalse;
    aStringCopy(line, "flags: ");
    if (pASTNode->flags & fConstant) {
      if (sep == aTrue)
        aStringCat(line, ",");
      aStringCat(line, "fConstant");
      sep = aTrue;
    }
    if (pASTNode->flags & fGlobal) {
      if (sep == aTrue)
        aStringCat(line, ",");
      aStringCat(line, "fGlobal");
      sep = aTrue;
    }
    if (pASTNode->flags & fUnsigned) {
      if (sep == aTrue)
        aStringCat(line, ",");
      aStringCat(line, "fUnsigned");
      sep = aTrue;
    }
    if (pASTNode->flags & fAddress) {
      if (sep == aTrue)
        aStringCat(line, ",");
      aStringCat(line, "fAddress");
      sep = aTrue;
    }
    if (pASTNode->flags & fByte) {
      if (sep == aTrue)
        aStringCat(line, ",");
      aStringCat(line, "fByte");
      sep = aTrue;
    }
    if (pASTNode->flags & fShort) {
      if (sep == aTrue)
        aStringCat(line, ",");
      aStringCat(line, "fShort");
      sep = aTrue;
    }
    if (pASTNode->flags & fVoid) {
      if (sep == aTrue)
        aStringCat(line, ",");
      aStringCat(line, "fVoid");
      sep = aTrue;
    }
    if (pASTNode->flags & fPushVal) {
      if (sep == aTrue)
        aStringCat(line, ",");
      aStringCat(line, "fPushVal");
      sep = aTrue;
    }
    if (pASTNode->flags & fBreakable) {
      if (sep == aTrue)
        aStringCat(line, ",");
      aStringCat(line, "fBreakable");
      sep = aTrue;
    }
    if (pASTNode->eType == tJumpStatement) {
      if (pASTNode->flags & fJumpReturn) {
        if (sep == aTrue)
          aStringCat(line, ",");
        aStringCat(line, "fJumpReturn");
        sep = aTrue;
      }
      if (pASTNode->flags & fJumpContinue) {
        if (sep == aTrue)
          aStringCat(line, ",");
        aStringCat(line, "fJumpContinue");
        sep = aTrue;
      }
      if (pASTNode->flags & fJumpBreak) {
        if (sep == aTrue)
          aStringCat(line, ",");
        aStringCat(line, "fJumpBreak");
        sep = aTrue;
      }
      if (pASTNode->flags & fJumpGoto) {
        if (sep == aTrue)
          aStringCat(line, ",");
        aStringCat(line, "fJumpGoto");
        sep = aTrue;
      }
    }
    if (pASTNode->eType == tSelectionStatement) {
      if (pASTNode->flags & fSelectIf) {
        if (sep == aTrue)
          aStringCat(line, ",");
        aStringCat(line, "fSelectIf");
        sep = aTrue;
      }
      if (pASTNode->flags & fSelectSwitch) {
        if (sep == aTrue)
          aStringCat(line, ",");
        aStringCat(line, "fSelectSwitch");
        sep = aTrue;
      }
    }

    aStream_WriteLine(aStreamLibRef(out), out, line, &astErr);
  }

  /* output the unique node id */
  if (astErr == aErrNone) {
    aStringCopy(line, indent);
    aStringCat(line, "id: ");
    aStringFromInt(num, pASTNode->count);
    aStringCat(line, num);
    aStream_WriteLine(aStreamLibRef(out), out, line, &astErr);
  }

  /* show the reference if present */
  if (pASTNode->pRef != NULL) {
    aStringCopy(line, indent);
    aStringCat(line, "ref: ");
    aStringFromInt(num, pASTNode->pRef->count);
    aStringCat(line, num);
    aStream_WriteLine(aStreamLibRef(out), out, line, &astErr);
  }

  /* output the node data */
  if (astErr == aErrNone) {

    switch (pASTNode->eType) {

    case tSelectionStatement:
    case tLabeledStatement:
      if (((pASTNode->eType == tSelectionStatement)
           && (pASTNode->flags & fSelectSwitch))
          || ((pASTNode->eType == tLabeledStatement)
              && (pASTNode->flags & (fLabelCase | fLabelDefault)))) {
        /* show the switch chain */
        if (pASTNode->t.switchChain.pNext != NULL) {
          aStringCopy(line, indent);
          aStringCat(line, "chain: ");
          aStringFromInt(num, pASTNode->t.switchChain.pNext->count);
          aStringCat(line, num);
          aStream_WriteLine(aStreamLibRef(out), out, line, &astErr);
        }
      }
      /* fall through */
    case tTypeSpecifier:
    case tIdentifier:
    case tConstant:
    case tString:
    case tDeclarator:
      if (pASTNode->pToken != NULL) {
        aStringCopy(line, "data: ");
        if ((!aStream_Write(aStreamLibRef(out), out, indent, indentChars, &astErr))
            && (!aStream_WriteLine(aStreamLibRef(out), out, line, &astErr))) {
          aStringCat(indent, "  ");
          aToken_Output(pASTNode->pToken, indent, out);
        }
      }
      break;

    case tInitDeclaratorList:
      aStringCopy(line, indent);
      aStringCat(line, "count: ");
      aStringFromInt(num, pASTNode->t.declarationList.stackSize);
      aStringCat(line, num);
      aStream_WriteLine(aStreamLibRef(out), out, line, &astErr);
      break;
      
    case tConstantValue:
    case tExpression:
      if (pASTNode->flags & fConstant) {
        aStringCopy(line, indent);
        aStringCat(line, "constant value: ");
        if (pASTNode->flags & fShort)
          aStringFromInt(num, pASTNode->v.shortVal);
        else
          aStringFromInt(num, pASTNode->v.byteVal);
        aStringCat(line, num);
        aStream_WriteLine(aStreamLibRef(out), out, line, &astErr);
      }
      break;

    case tOpcode:
      if (!aStream_Write(aStreamLibRef(out), out, indent, indentChars, &astErr)) 
        aStream_WriteLine(aStreamLibRef(out), out, "data:", &astErr);
      if ((astErr == aErrNone) 
          && !aStream_Write(aStreamLibRef(out), out, indent, indentChars, &astErr)) {
        char opText[10];
        aTEA_TextFromOpCode(opText, (tOpCode)pASTNode->t.opcode.asmCode.code[0]);
        aStream_WriteLine(aStreamLibRef(out), out, opText, &astErr);
      }
      break;

    case tAssignmentOperator:
      aStringCopy(line, indent);
      aStringCat(line, "op: ");
      if (pASTNode->flags & fAssignOpEqual)
        aStringCat(line, "'='");
      aStream_WriteLine(aStreamLibRef(out), out, line, &astErr);
      break;

    case tUnaryOperator:
      aStringCopy(line, indent);
      aStringCat(line, "op: ");
      if (pASTNode->flags & fUnaryOpMinus)
        aStringCat(line, "'-'");
      if (pASTNode->flags & fUnaryOpPlus)
        aStringCat(line, "'+'");
      if (pASTNode->flags & fUnaryOpBang)
        aStringCat(line, "'!'");
      if (pASTNode->flags & fUnaryOpTilde)
        aStringCat(line, "'~'");
      aStream_WriteLine(aStreamLibRef(out), out, line, &astErr);
      break;

    } /* eType switch */
  }

  /* visit the other nodes */
  if (pASTNode->pChildren)
    astErr = aASTNode_Output(pASTNode->pChildren, depth + 1, out);
  if ((astErr == aErrNone) && (pASTNode->pNext))
    aASTNode_Output(pASTNode->pNext, depth, out);
  
  return astErr;
  
} /* aASTNode_Output */

#endif /* aASTOUTPUT */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aASTNode_AddChild
 */

aErr aASTNode_AddChild(aASTNode* pNode, 
		       aASTNode* pChild)
{
  aErr astErr = aErrNone;

  aAssert(pNode);
  
  if ((astErr == aErrNone) && (pChild != NULL)) {
    if (pNode->pChildren == NULL)
      pNode->pChildren = pChild;
    else {
      aASTNode* pTemp = pNode->pChildren;
      while (pTemp->pNext != NULL)
        pTemp = pTemp->pNext;
      pTemp->pNext = pChild;
    }
    pChild->pParent = pNode;
  }
  
  return astErr;

} /* aASTNode_AddChild */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aASTNode_GetSymbolTable
 * 
 * looks for the first (deepest) scoping symbol table
 */

aSymbolTableRef aASTNode_GetSymbolTable(aASTNode* pNode,
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

} /* aASTNode_GetSymbolTable */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aASTNode_AddUniqueSymbol
 */

aErr aASTNode_AddUniqueSymbol(aAST* pAST,
			      aSymbolTableRef tableRef,
			      char* key,
			      aASTNodeSym* pSymData,
			      aASTNode* pNode)
{
  aErr astErr = aErrNone;
  aErr symErr = aErrNone;
  aASTNodeSym* pNewSymData;
  
  aAssert(pSymData);
  aAssert(tableRef);

  /* first, make sure the symbol has no yet been defined */
  aSymbolTable_Find(pAST->ioRef, tableRef, key, NULL, &symErr);
    
  if (symErr == aErrNotFound) {
    pNewSymData = (aASTNodeSym*)aMemAlloc(sizeof(aASTNodeSym));
    if (pNewSymData == NULL)
      astErr = aErrMemory;
    else {
      *pNewSymData = *pSymData;
      aSymbolTable_Insert(pAST->ioRef, tableRef, key, 
      			  pNewSymData, sASTNodeSymDelete,
      			  NULL, &astErr);
    }
  } else if (symErr == aErrNone) {
    char line[100];
    aStringCopy(line, aAST_SYM_REDEFINEDx);
    aStringCat(line, key);
    astErr = aAST_OutputError(pAST, pNode, line);
  }
  return astErr;

} /* aASTNode_AddUniqueSymbol */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aASTNode_Destroy
 */

void aASTNode_Destroy(aAST* pAST, aASTNode* pASTNode)
{
  /* destroy the token if it was being held on to */
  if (pASTNode->pToken != NULL) {
    aTokenizer_Dispose(pAST->ioRef, pAST->tokenizer, 
    		       pASTNode->pToken, NULL);
    pASTNode->pToken = NULL;
  }

  /* destroy the children first, then the siblings */
  if (pASTNode->pChildren)
    aASTNode_Destroy(pAST, pASTNode->pChildren);
  if (pASTNode->pNext)
    aASTNode_Destroy(pAST, pASTNode->pNext);

  /* clean up a symbol table if present */  
  if (pASTNode->symbolTable != NULL)
    aSymbolTable_Destroy(pAST->ioRef, pASTNode->symbolTable, NULL);

  /* then free ourselves */
  aMemPool_Free(pAST->ioRef, pAST->ASTNodePool, pASTNode, NULL);

} /* aASTNode_Destroy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aASTNode_Unwind
 * 
 * destroys the node and sub-nodes and restores tokens in order
 */

void aASTNode_Unwind(aAST* pAST, 
		     aASTNode* pASTNode)
{
  if (pASTNode->pNext) {
    aASTNode_Unwind(pAST, pASTNode->pNext);
    pASTNode->pNext = NULL;
  }
  
  if (pASTNode->pChildren) {
    aASTNode_Unwind(pAST, pASTNode->pChildren);
    pASTNode->pChildren = NULL;
  }

  if (pASTNode->pToken) {
    aTokenizer_PushBack(pAST->ioRef, pAST->tokenizer, 
    			pASTNode->pToken, NULL);
    pASTNode->pToken = NULL;
  }

  aASTNode_Destroy(pAST, pASTNode);

} /* aASTNode_Unwind */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aToken_Output
 *
 * Pretty prints the token to the specified output stream.
 *
 */

aErr aToken_Output(aToken* pToken,
		   const char* indent,
		   aStreamRef output)
{
  aErr ioErr = aErrNone;
  char line[100];
  char num[20];
  aTokenInfo ti;

  /* get information about the token */
  aToken_GetInfo(aStreamLibRef(output), pToken, &ti, &ioErr);

  aStringCopy(line, indent);
  aStringCat(line, "--- Token ---");
  aStream_WriteLine(aStreamLibRef(output), output, 
  		    line, &ioErr);

  /* output the file name */
  aStringCopy(line, indent);
  aStringCat(line, "file: \"");
  aStringCat(line, ti.pSourceName);
  aStringCat(line, "\"");
  aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);

  /* output the line number */
  aStringCopy(line, indent);
  aStringCat(line, "line: ");
  aStringFromInt(num, ti.nLine);
  aStringCat(line, num);
  aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);

  /* output the column number */
  aStringCopy(line, indent);
  aStringCat(line, "column: ");
  aStringFromInt(num, ti.nColumn);
  aStringCat(line, num);
  aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);

  /* build up the token data and display it */
  aStringCopy(line, indent);
  aStringCat(line, "type: ");
  switch (pToken->eType) {

  case tkString:
    aStringCat(line, "string");
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    aStringCopy(line, indent);
    aStringCat(line, "characters: ");
    aStringCat(line, pToken->v.string);
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    break;

  case tkInt:
    aStringCat(line, "int");
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    aStringCopy(line, indent);
    aStringCat(line, "value: ");
    aStringFromInt(num, pToken->v.integer);
    aStringCat(line, num);
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    break;

  case tkPreProc:
    aStringCat(line, "pre-processor");
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    aStringCopy(line, indent);
    aStringCat(line, "directive: ");
    aStringCat(line, pToken->v.identifier);
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    break;

  case tkIdentifier:
    aStringCat(line, "identifier");
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    aStringCopy(line, indent);
    aStringCat(line, "value: ");
    aStringCat(line, pToken->v.identifier);
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    break;

  case tkSpecial:
    aStringCat(line, "special");
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    aStringCopy(line, indent);
    aStringCat(line, "character: ");
    num[0] = pToken->v.special;
    num[1] = 0;
    aStringCat(line, num);
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    break;

  case tkNewLine:
    aStringCat(line, "newline");
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    break;

  default:
    aStringCat(line, "invalid");
    aStream_WriteLine(aStreamLibRef(output), output, line, &ioErr);
    break;

  } /* pToken->eType switch */

  return(ioErr);

} /* end of aToken_Output */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aASTNodeSymDelete
 */

aErr sASTNodeSymDelete(void* pData, void* ref)
{
  if (pData != NULL)
    aMemFree((aMemPtr)pData);
  return aErrNone;

} /* aASTNodeSymDelete */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sASTGetLeftMostToken
 */

aToken* sASTGetLeftMostToken(const aASTNode* pNode)
{
  if (!pNode)
    return NULL;
  if (pNode->pToken)
    return pNode->pToken;
  else if (pNode->pChildren)
    return sASTGetLeftMostToken(pNode->pChildren);
  else if (pNode->pNext)
    return sASTGetLeftMostToken(pNode->pNext);
  
  return NULL;

} /* sASTGetLeftMostToken */


#ifdef aASTOUTPUT

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_Output
 */

aErr aAST_Output(aAST* pAST,
		 aStreamRef out,
		 aStreamRef err,
		 aStreamRef result)
{
  aErr astErr = aErrNone;

  if (pAST->pTree)
    astErr = aASTNode_Output(pAST->pTree, 0, result);

  return astErr;

} /* aAST_Output */

#endif /* aLASTOUTPUT */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_OutputError
 */

aErr aAST_OutputError(void* vpAST, 
		      aASTNode* pNode,
		      const char* errorDesc)
{
  aErr astErr = aErrNone;
  aAST* pAST = (aAST*)vpAST;

  aAssert(pAST);

  pAST->nErrors++;
  
  if (pAST->errStream != NULL) {

    /* scan down for the node containing the error token */
    aToken* pErrToken = sASTGetLeftMostToken(pNode);

    if (pAST->errProc) {
      const char* data[2];
      unsigned int line = 0;
      unsigned int column = 0;
      aTokenInfo ti;

      data[0] = "unknown";
      data[1] = (char*)errorDesc;

      if (pErrToken) {
        aToken_GetInfo(aStreamLibRef(pAST->errStream), 
        	       pErrToken, &ti, &astErr);
        if (astErr == aErrNone) {
          data[0] = ti.pSourceName;
          line = ti.nLine;
          column = ti.nColumn;
        }
      }
      astErr = pAST->errProc(tkErrCompile, line, column, 
      			     2, data, pAST->errProcRef);
    }
  }

  return astErr;

} /* aAST_OutputError */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_ParseSpecialChar
 */

aErr aAST_OutputNextTokenError(aAST* pAST, 
		               const char* errorDesc)
{
  aErr astErr = aErrNone; 
  aASTNode* pTemp;
  aToken* pToken;

  /* get the next token */
  if ((astErr == aErrNone)
      && !aTokenizer_Next(pAST->ioRef, 
      			  pAST->tokenizer, 
      			  &pToken, &astErr)) {
    
    /* build a temp node to report the error */
    astErr = aASTNode_Create(pAST, tString, &pTemp);
    if (astErr == aErrNone) {
      pTemp->pToken = pToken;
      astErr = aAST_OutputError(pAST, pTemp, errorDesc);
      pTemp->pToken = NULL;
    }
    if (astErr == aErrNone)
      aASTNode_Destroy(pAST, pTemp);
    if (astErr == aErrNone) {
      aTokenizer_PushBack(pAST->ioRef, pAST->tokenizer, 
      			  pToken, &astErr);
    }
  }

  return astErr;

} /* aAST_OutputNextTokenError */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_ParseSpecialChar
 *
 * looks at the next token to see if it is the character expected
 *
 * returns true if found (and gobbles up the character token)
 *
 */

aBool aAST_ParseSpecialChar(aAST* pAST,
			    char special,
			    aBool bRequired,
			    aErr* pErr)
{
  aBool bMatched = aFalse;
  aToken* pToken;
  aASTNode* pTemp;
  char errMsg[100];

  /* get the next token */
  if (*pErr == aErrNone) {
    if (aTokenizer_Next(pAST->ioRef, 
      		        pAST->tokenizer, 
      		        &pToken, pErr)) {
      /* tell the user it was missing if required */
      if ((*pErr == aErrEOF) && (bRequired == aTrue)) {
        errMsg[0] = special;
        errMsg[1] = 0;
        aStringCat(errMsg, aAST_CHARACTER_EXPECTED);
        *pErr = aASTNode_Create(pAST, tString, &pTemp);
        if (*pErr == aErrNone)
          *pErr = aAST_OutputError(pAST, pTemp, errMsg);
        if (*pErr == aErrNone)
          aASTNode_Destroy(pAST, pTemp);
        *pErr = aErrParse;
      }
    } else {

      /* we got a token so see if it is the asm start */
      if ((pToken->eType != tkSpecial)
          || (pToken->v.special != special)) {

        /* tell the user it was missing if required */
        if (bRequired == aTrue) {
          errMsg[0] = special;
          errMsg[1] = 0;
          aStringCat(errMsg, aAST_CHARACTER_EXPECTED);
          *pErr = aASTNode_Create(pAST, tString, &pTemp);
          if (*pErr == aErrNone) {
            pTemp->pToken = pToken;
            *pErr = aAST_OutputError(pAST, pTemp, errMsg);
            pTemp->pToken = NULL;
          }
          if (*pErr == aErrNone)
            aASTNode_Destroy(pAST, pTemp);
        }

        /* push it back if it is not the right token */
        aTokenizer_PushBack(pAST->ioRef, pAST->tokenizer, 
      			    pToken, pErr);
      } else {
        aTokenizer_Dispose(pAST->ioRef, pAST->tokenizer, 
      			   pToken, pErr);
        bMatched = aTrue;
      }
    }
  }

  return bMatched;

} /* aAST_ParseSpecialChar */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_ParseIdentifierToken
 */

aBool aAST_ParseIdentifierToken(aAST* pAST,
			        const char* identifier,
			        const aBool bRequired,
			        aErr* pErr)
{
  aBool bMatched = aFalse;
  aToken* pToken;

  /* get the next token */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pAST->ioRef, pAST->tokenizer, 
      			  &pToken, NULL)) {

    /* see if it is the requested identifier */
    if ((pToken->eType != tkIdentifier)
        || (aStringCompare(pToken->v.identifier, identifier) != 0)) {
      /* push it back if it is not the right token */
      aTokenizer_PushBack(pAST->ioRef, pAST->tokenizer, pToken, pErr);
    } else {
      aTokenizer_Dispose(pAST->ioRef, pAST->tokenizer, pToken, pErr);
      bMatched = aTrue;
    }
  }

  if ((*pErr == aErrNone)
      && (bMatched == aFalse) 
      && (bRequired == aTrue)) {
    char errMsg[100];
    aStringCopy(errMsg, "\"");
    aStringCat(errMsg, identifier);
    aStringCat(errMsg, "\"");
    aStringCat(errMsg, aAST_IDENT_EXPECTED);
    *pErr = aAST_OutputError(pAST, NULL, errMsg);
  }

  return bMatched;

} /* aAST_ParseIdentifierToken */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_ParseDeclarationSpecifier
 */

aBool aAST_ParseDeclarationSpecifier(aAST* pAST, 
				     aASTNode** ppNewTree,
				     aErr* pErr)
{
  aBool bMatched = aFalse;
  aASTNode* pTypeSpecifier;
  aSFlags flags = 0;

  *ppNewTree = NULL;

  while ((*pErr == aErrNone) 
         && aAST_ParseTypeSpecifier(pAST, &pTypeSpecifier, pErr)) {
    bMatched = aTrue;

    /* if there was a match, build up the node */
    if ((*pErr == aErrNone) && (*ppNewTree == NULL))
      *pErr = aASTNode_Create(pAST, tDeclarationSpecifier, ppNewTree);

    if (*pErr == aErrNone)
      *pErr = aASTNode_AddChild(*ppNewTree, pTypeSpecifier);
      
    if (*pErr == aErrNone) {
      if ((flags & fBaseTypeMask) 
          && (pTypeSpecifier->flags & fBaseTypeMask)) {
        if (*pErr == aErrNone)
          *pErr = aAST_OutputError(pAST, pTypeSpecifier, 
          			   aAST_TYPE_ALREADY_SPECIFIED);
      } else
        flags |= pTypeSpecifier->flags;
    }
  }

  /* store the accumulated type flags here for the special case
   * of void parameters */
  if (bMatched)
    (*ppNewTree)->flags |= flags;
    
  return bMatched;

} /* aAST_ParseDeclarationSpecifier */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_ParseConstant
 */

aBool aAST_ParseConstant(aAST* pAST,
			 aASTNode** ppNewTree,
			 aErr* pErr)
{
  aBool bMatched = aFalse;
  aToken* pToken;

  /* check for a constant */
  if ((*pErr == aErrNone)
      && !aTokenizer_Next(pAST->ioRef, pAST->tokenizer, 
      			  &pToken, NULL)) {
    if (pToken->eType == tkInt)  {
      *pErr = aASTNode_Create((aAST*)pAST, tConstant, ppNewTree);
      if (*pErr == aErrNone) {
        (*ppNewTree)->flags |= fConstant;
        (*ppNewTree)->pToken = pToken;
        bMatched = aTrue;
      }
    } else {
      aTokenizer_PushBack(pAST->ioRef, pAST->tokenizer, pToken, pErr);
    }
  }

  return bMatched;

} /* aAST_ParseConstant */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_ParseTypeSpecifier
 */

aBool aAST_ParseTypeSpecifier(aAST* pAST, 
			      aASTNode** ppNewTree,
			      aErr* pErr)
{
  aBool bMatched = aFalse;

  /* check for a proper type specifier */
  if (*pErr == aErrNone) {
    aToken* pToken = NULL;
    aSFlags setflags = 0;
    aSFlags clearedflags = 0;

    if (!aTokenizer_Next(pAST->ioRef, pAST->tokenizer, 
    			 &pToken, NULL)) {
      if (pToken->eType == tkIdentifier) {
        /* look for a proper type reserved word */
        if (!aStringCompare(pToken->v.identifier, aAST_VOID))
          setflags |= fVoid;
        else if (!aStringCompare(pToken->v.identifier, aAST_CHAR))
          setflags |= fByte;
        else if (!aStringCompare(pToken->v.identifier, aAST_INT))
          setflags |= fShort;
        else if (!aStringCompare(pToken->v.identifier, aAST_STRING))
          setflags |= fString;
        else if (!aStringCompare(pToken->v.identifier, aAST_UNSIGNED))
          setflags |= fUnsigned;
        else if (!aStringCompare(pToken->v.identifier, aAST_SIGNED))
          clearedflags |= fUnsigned;

        /* if found, build the node and set the type flag */
        if (setflags || clearedflags) {
          setflags &= ~clearedflags;
          *pErr = aASTNode_Create(pAST, 
          			  tTypeSpecifier, ppNewTree);
          if (*pErr == aErrNone) {
            bMatched = aTrue;
            (*ppNewTree)->pToken = pToken;
            (*ppNewTree)->flags |= setflags;
          }
        }
      }
      if ((*pErr == aErrNone)
          &&(bMatched == aFalse))
        aTokenizer_PushBack(pAST->ioRef, pAST->tokenizer, 
        		    pToken, pErr);
    }
  }

  return bMatched;

} /* aAST_ParseTypeSpecifier */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_PushUpFlags
 */

aErr aAST_PushUpFlags(aASTNode* pParent, 
		      aASTNode* pChild)
{
  aErr astErr = aErrNone;

  /* if pushing type, do it */
  if (pChild->flags & fGetType) {
    aSFlags f = (aSFlags)(pChild->flags & fTypeMask);
    f &= ~fAddress; /* don't push up the address flag */
    pParent->flags |= f;
  }

  /* push up the constant if present */
  if (pChild->flags & fConstant) {
    pParent->flags |= fConstant;
    if (pChild->flags & fByte)
      pParent->v.byteVal = pChild->v.byteVal;
    else if (pChild->flags & fShort)
      pParent->v.shortVal = pChild->v.shortVal;
  }

  return astErr;

} /* aAST_PushUpFlags */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_OptConstant
 */

aErr aAST_OptConstant(aAST* pAST,
		      aASTNode* pConstant,
		      aSFlags flags)
{
  aErr astErr = aErrNone;
  int val;

  aAssert(pConstant);
  aAssert(pConstant->eType == tConstant);
  
  pConstant->flags |= flags;
  val = pConstant->pToken->v.integer;

  /* check for range errors */
  if (pConstant->flags & fByte) {
    if ((val < tBYTE_MIN) || 
        (val > tBYTE_MAX)) {
      astErr = pAST->outputErrProc(pAST, pConstant, 
      				   aAST_VAL_EXCEEDS_BYTE);
    } else {
      pConstant->v.byteVal = (tBYTE)val;
    }
  } else if (pConstant->flags & fShort) {
    if ((val < tSHORT_MIN) || 
        (val > tSHORT_MAX)) {
      astErr = pAST->outputErrProc(pAST, pConstant, 
      				   aAST_VAL_EXCEEDS_SHORT);
    } else {
      pConstant->v.shortVal = (tSHORT)val;
    }
  } else {
    if ((val < tBYTE_MIN) || 
        (val > tBYTE_MAX)) {
      pConstant->v.shortVal = (tSHORT)val;
      pConstant->flags |= fShort;
    } else {
      pConstant->v.byteVal = (tBYTE)val;
      pConstant->flags |= fByte;
    }
  }

  return astErr;

} /* aAST_OptConstant */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAST_OptDeclarationSpecifier
 */

aErr aAST_OptDeclarationSpecifier(aAST* pAST, 
			          aASTNode* pDeclarationSpecifier)
{
  aErr astErr = aErrNone;
  aASTNode* pChild;

  aAssert(pAST);
  aAssert(pDeclarationSpecifier->eType == tDeclarationSpecifier);
  
  pChild = pDeclarationSpecifier->pChildren;

  /* snag the flags from the type specifiers */
  while (pChild && (astErr == aErrNone)) {
    pDeclarationSpecifier->flags |= pChild->flags;
    pChild = pChild->pNext;
  }
  
  return astErr;

} /* aAST_OptDeclarationSpecifier */
