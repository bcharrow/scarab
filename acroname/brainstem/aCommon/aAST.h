/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aAST.h 						   */
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

#ifndef _aAST_H_
#define _aAST_H_

#include "aIO.h"
#include "aTEA.h"

typedef enum {
  tTranslationUnit,
  tExternalDeclaration,
  tFunctionDefinition,
  tDeclarationSpecifier,
  tTypeSpecifier,
  tInitDeclaratorList,
  tInitDeclarator,
  tDeclarator,
  tParameterList,
  tParameterDeclaration,
  tCompoundStatement,
  tDeclarationList,
  tDeclaration,
  tStatementList,
  tStatement,
  tAsmStatement,
  tJumpStatement,
  tAsmList,
  tOpcode,
  tExpression,
  tConstant,
  tString,
  tIdentifier,
  tLabeledStatement,
  tExpressionStatement,
  tAssignmentExpression,
  tUnaryExpression,
  tAssignmentOperator,
  tPostFixExpression,
  tArgumentExpressionList,
  tPrimaryExpression,
  tSelectionStatement,
  tInitializer,
  tUnaryOperator,
  tCastExpression,
  tConditionalExpression,
  tLogicalORExpression,
  tLogicalANDExpression,
  tInclusiveORExpression,
  tExclusiveORExpression,
  tANDExpression,
  tEqualityExpression,
  tRelationalExpression,
  tShiftExpression,
  tAdditiveExpression,
  tMultiplicativeExpression,
  tLoopStatement,
  
  tReflexUnit,
  tReflexDeclaration,
  tModuleDeclaration,
  tMessageDeclaration,
  tVectorDeclaration,
  tInlineDeclaration,
  tModuleSpecifier,
  tMessageSpecifier,
  tVectorSpecifier,
  tPacketData,
  tMessageReferenceList,
  tMessageReference,
  tLModifyMessageReference,
  tRModifyMessageReference,
  tMessageModifyReference,
  tConstantValue

} aASTType;


#define fBaseTypeMask	0x000000F8
#define fTypeMask	0x000000FC
#define fConstant	0x00000001
#define fGlobal		0x00000002
#define fUnsigned	0x00000004
#define fAddress	0x00000008
#define fByte		0x00000010
#define fShort		0x00000020
#define fVoid		0x00000040
#define	fString		0x00000080
#define fPushVal	0x00000100
#define fGetType	0x00000200
#define fBreakable	0x00000400

/* node-specific jump flags */
#define fFunctionUsed	0x00010000

#define fIdentRtnName	0x00010000
#define fIdentAsmCall	0x00020000
#define fIdentLocalVar	0x00040000
#define fIdentInited	0x00080000

#define fJumpReturn	0x00010000
#define fJumpContinue	0x00020000
#define fJumpBreak	0x00040000
#define fJumpGoto	0x00080000

#define fSelectIf	0x00010000
#define fSelectSwitch	0x00020000

#define f2OpOR		0x00010000
#define f2OpXOR		0x00020000
#define f2OpAND		0x00040000
#define f2OpPLUS	0x00080000
#define f2OpMINUS	0x00100000
#define f2OpMULT	0x00200000
#define f2OpDIV		0x00400000
#define f2OpMOD		0x00800000

#define fEquality	0x00010000
#define fNotEquality	0x00020000

#define fRelateLT	0x00010000
#define fRelateGT	0x00020000
#define fRelateEQ	0x00040000

#define fShiftLeft	0x00010000
#define fShiftRight	0x00020000

#define fUnaryPreInc	0x00010000
#define fUnaryPreDec	0x00020000

#define fPostFixInc	0x00010000
#define fPostFixDec	0x00020000
#define fPostFixRoutine	0x00040000

#define fLoopWhile	0x00010000
#define fLoopFor	0x00020000
#define fLoopDoWhile	0x00040000
#define fLoopPre	0x00080000
#define fLoopCondition	0x00100000
#define fLoopPost	0x00200000

#define fAssignOpEqual	0x00010000

#define fUnaryOpMinus	0x00010000
#define fUnaryOpPlus	0x00020000
#define fUnaryOpBang	0x00040000
#define fUnaryOpTilde	0x00080000

#define fLabelCase	0x00010000
#define fLabelDefault	0x00020000
#define fLabelGeneral	0x00040000

#define fLModMinus	0x00010000
#define fLModEqual	0x00020000

#define fRModPlus	0x00010000
#define fRModMinus	0x00020000
#define fRModMult	0x00040000
#define fRModRShift	0x00080000

#define fInlinePfx	0x00010000
#define fInlineSfx	0x00020000

typedef long aSFlags;

typedef struct aASTNodeAsmData {
  char		opLen;
  char		code[aTEA_MAXOPCODESIZE];
} aASTNodeAsmData;

typedef struct aASTFunction {
  unsigned char		callSize;
  unsigned char		stackSize;
} aASTFunction;

typedef struct aASTOpcode {
  aASTNodeAsmData	asmCode;
} aASTOpcode;

typedef struct aASTDeclarationList {
  unsigned char		stackSize;
} aASTDeclarationList;

typedef struct aASTIdentifier {
  unsigned char		stackPos;
} aASTIdentifier;

typedef struct aASTParameterList {
  unsigned char		stackSize;
} aASTParameterList;

typedef struct aASTArgumentList {
  char			stackSize;
} aASTArgumentList;

typedef struct aASTString {
  unsigned char		length;
} aASTString;

typedef struct aASTSwitch {
  tADDRESS		switchEnd; /* must be first in struct */
  struct aASTNode*	pNext;
} aASTSwitch;

typedef struct aASTLoop {
  tADDRESS		loopEnd; /* must be first in struct */
  tADDRESS		loopEval;
  tADDRESS		loopContinue;
} aASTLoop;

typedef struct aASTModule {
  tBYTE			address; /* must be first in struct */
  struct aASTNode*	pNext;
} aASTModule;

typedef struct aASTNode {
#ifdef aASTOUTPUT
  unsigned short	count;
#endif
  aASTType		eType;
  aSFlags		flags;
  union {
    aASTFunction	function;
    aASTOpcode		opcode;
    aASTDeclarationList	declarationList;
    aASTIdentifier	identifier;
    aASTParameterList	parameterList;
    aASTArgumentList	argumentList;
    aASTString		string;
    aASTSwitch		switchChain;
    aASTLoop		loop;
    aASTModule		module;
  } t;
  union {
   tSHORT		shortVal;
   tBYTE		byteVal;
  } v;
  aSymbolTableRef	symbolTable;
  aToken*		pToken;
  struct aASTNode*	pRef;
  struct aASTNode*	pNext;
  struct aASTNode*	pChildren;
  struct aASTNode*	pParent;
  tADDRESS		nCodePosition;
} aASTNode;



/* tree management procs */
typedef aErr (*ASTNodeCreateProc)(void* vpAST, 
				   aASTType eType, 
				   aASTNode** ppNode);
typedef aErr (*ASTNodeAddChildProc)(aASTNode* pNode, 
				     aASTNode* pChild);

typedef aErr (*OutputErrorProc)(void* vpAST, 
				aASTNode* pNode, 
				const char* errorDesc);
				
#define aASTTREEHEADER  aIOLib			ioRef;		    \
			aTokenizerRef		tokenizer;	    \
			aMemPoolRef		ASTNodePool;	    \
			aStreamRef		errStream;	    \
			aStreamRef		outputStream;       \
  			aASTNode*		pTree;		    \
			int			nErrors;	    \
		  	aTokenErrProc 		errProc;	    \
		  	void* 			errProcRef;	    \
			OutputErrorProc		outputErrProc

typedef struct aAST {
  aASTTREEHEADER;
} aAST;

#define aASTNODEPOOLSIZE	64

typedef enum {
  stLabel,
  stSymbol
} aASTNodeSymType;

typedef struct aASTNodeSym {
  aASTNodeSymType	eType;
  aASTNode*		pNode;
} aASTNodeSym;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aASTNode management functions
 */

#ifdef __cplusplus
extern "C" {
#endif

aErr aASTNode_Create(aAST* pAST,
		     aASTType eType,
		     aASTNode** ppNode);
#ifdef aASTOUTPUT
aErr aASTNode_Output(aASTNode* pASTNode, 
		     const int depth,
		     aStreamRef out);
#endif /* aASTOUTPUT */

aErr aASTNode_AddChild(aASTNode* pNode, 
			      aASTNode* pChild);
aSymbolTableRef aASTNode_GetSymbolTable(aASTNode* pNode,
				      	aASTNode** ppScopeNode);
aErr aASTNode_AddUniqueSymbol(aAST* pAST,
			      aSymbolTableRef tableRef,
			      char* key,
			      aASTNodeSym* pSymData,
			      aASTNode* pNode);
void aASTNode_Destroy(aAST* pAST, 
		      aASTNode* pASTNode);
void aASTNode_Unwind(aAST* pAST, 
		     aASTNode* pASTNode);



#ifdef aASTOUTPUT
aErr aAST_Output(aAST* pAST,
		 aStreamRef out,
		 aStreamRef err,
		 aStreamRef result);
#endif /* aASTOUTPUT */

aErr aAST_OutputError(void* vpAST, 
		      aASTNode* pNode,
		      const char* errorDesc);

aErr aAST_OutputNextTokenError(aAST* pAST, 
		               const char* errorDesc);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Token utility routines
 */

aErr aToken_Output(aToken* pToken,
		   const char* indent,
		   aStreamRef output);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Common parsing utility functions
 */

aBool aAST_ParseSpecialChar(aAST* pAST,
			    char special,
			    aBool bRequired,
			    aErr* pErr);
aBool aAST_ParseIdentifierToken(aAST* pAST,
				const char* identifier,
				const aBool bRequired,
				aErr* pErr);
aBool aAST_ParseDeclarationSpecifier(aAST* pAST, 
				     aASTNode** ppNewTree,
				     aErr* pErr);


aBool aAST_ParseInclusiveORExpression(aAST* pAST,
				      aASTNode** ppNewTree,
				      aErr* pErr);
aBool aAST_ParseExclusiveORExpression(aAST* pAST,
				      aASTNode** ppNewTree,
				      aErr* pErr);
aBool aAST_ParseANDExpression(aAST* pAST,
			      aASTNode** ppNewTree,
			      aErr* pErr);
aBool aAST_ParseConstant(aAST* pAST,
			 aASTNode** ppNewTree,
			 aErr* pErr);
aBool aAST_ParseTypeSpecifier(aAST* pAST, 
			      aASTNode** ppNewTree,
			      aErr* pErr);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Common optimization utility functions
 */

aErr aAST_PushUpFlags(aASTNode* pParent, 
		      aASTNode* pChild);

aErr aAST_OptConstant(aAST* pAST,
		      aASTNode* pConstant,
		      aSFlags flags);

aErr aAST_OptDeclarationSpecifier(aAST* pAST, 
			          aASTNode* pDeclarationSpecifier);

#ifdef __cplusplus
}
#endif

#endif /* _aAST_H_ */
