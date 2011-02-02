/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aLAST.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of an LAST (annotated syntax tree) for  */
/*		the LEAF compiler. 		                   */
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

#ifndef _aLAST_H_
#define _aLAST_H_

#include "aAST.h"

#define aNUMMESSAGES	128
#define aNUMVECTORS	128
#define aNUMVECTORREFS	  8

typedef enum aLASTNodeTypeType {
  typeVOID = 0,
  typeCHAR,
  typeINT
} aLASTNodeTypeType;

typedef struct aLASTNodeAsmData {
  char		opLen;
  char		code[aTEA_MAXOPCODESIZE];
} aLASTNodeAsmData;

typedef aErr (*astVisitProc)(void* vpLAST, 
			     void* vpNode,
			     aBool bPreVisit);

typedef struct aMessageData {
  aBool bUsed;
  char data[16];
} aMessageData;

typedef struct aVectorData {
  aBool bUsed;
  char data[aNUMVECTORREFS][2];
} aVectorData;

typedef struct aReflexData {
  int			nMessages;
  int			nVectors;
  aMessageData		message[aNUMMESSAGES];
  aVectorData		vector[aNUMVECTORS];
} aReflexData;

typedef struct aLAST {
  aASTTREEHEADER;
  int			nPass;
  aReflexData*		modules[126];
  aASTNode*		pCurModule;
} aLAST;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * routines 
 */

aErr aLAST_Create(aIOLib ioRef,
		  aLAST** ppLAST,
		  aTokenErrProc errProc,
		  void* errProcRef);
aErr aLAST_Parse(aLAST* pLAST,
		aTokenizerRef tokenizerRef,
		aStreamRef errStream);
aErr aLAST_Optimize(aLAST* pLAST,
		    aStreamRef errStream);
aErr aLAST_Generate(aLAST* pLAST,
		    aStreamRef outputStream);
aErr aLAST_Destroy(aLAST* pLAST);


#endif /* _aLAST_H_ */
