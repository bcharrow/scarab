/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSAST.h							   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of an SAST (annotated syntax tree) for  */
/*		the TEA compiler. 		                   */
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

#ifndef _aSAST_H_
#define _aSAST_H_

#include "aTEA.h"
#include "aAST.h"

typedef enum aSASTNodeTypeType {
  typeVOID = 0,
  typeCHAR,
  typeINT
} aSASTNodeTypeType;

typedef aErr (*astVisitProc)(void* vpSAST, 
			     void* vpNode,
			     aBool bPreVisit);



typedef struct aSAST {
  aASTTREEHEADER;
  unsigned char*	opLengths;
  aBool			bResolved;
  int			nPass;
  tADDRESS		nCodeBytes;
  aASTNode*		pCurrentFunction;
  aBool			bJustCompute;
  tADDRESS		nCurRoutineStart;
  tSTACK		nStackOffset;
  aASTNode*		pMainRoutine;
  tSTACK		nGlobalStackSize;
  aBool			bInDeclarationList;
  aASTNode*		pLastStatement;
  char*			code;
  ASTNodeCreateProc	createNode;
  ASTNodeAddChildProc	addNodeChild;
} aSAST;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * routines 
 */

aErr aSAST_Create(aIOLib ioRef,
		  aSAST** ppSAST,
		  unsigned char* pOpLengths,
		  aTokenErrProc errProc,
		  void* errProcRef);
aErr aSAST_Parse(aSAST* pSAST,
		aTokenizerRef tokenizerRef,
		aStreamRef errStream);

aErr aSAST_Destroy(aSAST* pSAST);


#endif /* _aSAST_H_ */
