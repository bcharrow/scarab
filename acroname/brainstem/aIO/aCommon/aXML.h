/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aXML.c                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Cross-Platform implementation of XML parsing       */
/*              routines.                                          */
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

#ifndef _aXML_H_
#define _aXML_H_

#include "aMemPoolInternal.h"
#include "aTokenInternal.h"
#include "aIO.h"


typedef struct aXML {
  aIOLib		ioRef;
  aMemPool*		pNodePool;
  aTokenizer*		pTokenizer;
  struct aXMLNode*	pTree;
  aStreamRef		data;

  aTokenErrProc		errorProc;
  void*			errorRef;

  aXMLHandleStart	startProc;
  aXMLHandleContent	contentProc;
  aXMLHandleEnd		endProc;
  void*			procRef;

  unsigned int		nErrors;
  int			check;
} aXML;

#define aXMLCHECK	0x4444

#define aVALIDXML(p)						   \
  if ((p == NULL) ||						   \
      (((aXML*)p)->check != aXMLCHECK)) {		   		   \
    err = aErrParam;						   \
  }

typedef struct aXMLNode {
  aXML*			pXML;
  aToken* 		pKeyToken;
  aTokenList*		pData;
  struct aXMLNode*	pParent;
  struct aXMLNode*	pNext;
  struct aXMLNode*	pChildren;
} aXMLNode;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * static private routines
 */

static aErr sXMLNode_Create(aXML* pXML,
			    aToken* pKeyToken,
		     	    aXMLNode** ppNode);
static aErr sXMLNode_Destroy(aXMLNode* pNode);
static aErr sXMLNode_Parse(aXMLNode* pNode);
static aErr sXMLNode_Traverse(aXMLNode* pNode);
static aErr sXMLNode_AddChild(aXMLNode* pNode, aXMLNode* pChild);
static aErr sXMLNode_HandleData(aXMLNode* pNode,
			        aXMLHandleContent contentProc,
			        void* vpProcRef);

static aErr sXML_Parse(aXML* pXML);

#endif /* _aXML_H_ */
