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

#include "aXML.h"
#include "aIOInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sXMLNode_Create
 */

aErr sXMLNode_Create(aXML* pXML,
		     aToken* pKeyToken,
		     aXMLNode** ppNode)
{
  aErr err = aErrNone;
  
  aAssert(pXML);
  aAssert(ppNode);
  aAssert(pXML->pNodePool);
  aAssert(pXML->pTokenizer);
  aAssert(pXML->pTokenizer->pTokenPool);

  err = aMemPoolInternal_Alloc(pXML->pNodePool,
  				 (void**)ppNode);

  /* initialize if we get one */  				 
  if (err == aErrNone) {
    aBZero(*ppNode, sizeof(aXMLNode));
    (*ppNode)->pXML = pXML;
    (*ppNode)->pKeyToken = pKeyToken;
  }

  return err;

} /* sXMLNode_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sXMLNode_Destroy
 */

aErr sXMLNode_Destroy(aXMLNode* pNode)
{
  aErr err = aErrNone;
  
  if (pNode) {

    if (pNode->pChildren) {
      err = sXMLNode_Destroy(pNode->pChildren);
      pNode->pChildren = NULL;
    }

    if ((err == aErrNone) && pNode->pNext) {
      err = sXMLNode_Destroy(pNode->pNext);
      pNode->pNext = NULL;
    }

    if ((err == aErrNone) && pNode->pData) {
      /* MRW stash ioRef so we can delete streams attached to tokens */
      pNode->pData->pRef = (void*)pNode->pXML->ioRef;
      aTokenList_Destroy(pNode->pData);
      pNode->pData = NULL;
    }
    
    if ((err == aErrNone) && pNode->pKeyToken) {
      aTokenizerInternal_Dispose(pNode->pXML->pTokenizer, 
      				 pNode->pKeyToken);
      pNode->pKeyToken = NULL;
    }

    if (err == aErrNone)
      aMemPoolInternal_Free(pNode->pXML->pNodePool, pNode);
  }

  return err;

} /* sXMLNode_Destroy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sXMLNode_Parse
 */

aErr sXMLNode_Parse(aXMLNode* pNode)
{
  aErr err = aErrNone;
  aBool bClosed = aFalse;
  aXML* pXML = pNode->pXML;
  aToken* pToken = NULL;
  aToken* pKey = NULL;
  aToken* pClose = NULL;
  aToken* pLast = NULL;
  tkError tokenErrType;

  aAssert(pNode);

  while ((err == aErrNone) 
  	 && !bClosed 
  	 && !pNode->pXML->nErrors) {

    /* get another token */    
    err = aTokenizerInternal_Next(pXML->pTokenizer, &pToken);

    /* if none, report error */
    if (err != aErrNone) {

      /* the root case is an exeption and has not key token */
      if (pNode->pKeyToken || (err != aErrEOF))
        goto parse_error;
    }

    /* now, look for a key and parse if we have one */
    pLast = pToken;
    if ((err == aErrNone)
    	&& (pToken->eType == tkSpecial)
    	&& (pToken->v.special == '<')) {

      err = aTokenizerInternal_Next(pXML->pTokenizer, &pKey);
      if (err != aErrNone)
        /* expected a key */
        goto parse_error;
      
      /* if it is a close, toss the '/' token and get another */
      pLast = pKey;
      if ((pKey->eType == tkSpecial) && (pKey->v.special == '/')) {
        bClosed = aTrue;
        aTokenizerInternal_Dispose(pXML->pTokenizer, pKey);
        err = aTokenizerInternal_Next(pXML->pTokenizer, &pKey);
        if (err != aErrNone)
          /* expected a key */
          goto parse_error;
      }

      /* here, we should have an identifier */
      pLast = pKey;
      if (pKey->eType != tkIdentifier)
        goto parse_error;

      if (bClosed && aStringCompare(pNode->pKeyToken->v.identifier,
      				    pKey->v.identifier))
        /* close doesn't match */
        goto parse_error;
        
      err = aTokenizerInternal_Next(pXML->pTokenizer, &pClose);
      if (err != aErrNone) {
        /* expected a close bracket */
        tokenErrType = tkErrXMLMissingClose;
        goto parse_error;
      }

      pLast = pClose;
      if ((pClose->eType != tkSpecial) 
      	  || (pClose->v.special != '>')) {
        /* expected a close bracket */
        tokenErrType = tkErrXMLMissingClose;
        goto parse_error;
      }

      /* here, we have a tag, if it is a close we are done, if 
       * not, we need to recurse */
      if (!bClosed) {
        aXMLNode* pNew;
        err = sXMLNode_Create(pXML, pKey, &pNew);
        if (err == aErrNone)
          err = sXMLNode_AddChild(pNode, pNew);
        if (err == aErrNone)
          err = sXMLNode_Parse(pNew);

      } else {
        aTokenizerInternal_Dispose(pXML->pTokenizer, pKey);
      }
      aTokenizerInternal_Dispose(pXML->pTokenizer, pToken);
      aTokenizerInternal_Dispose(pXML->pTokenizer, pClose);

    } else {

      /* build the data list if it isn't in place */    
      if ((err == aErrNone) && !pNode->pData) {
        err = aTokenList_Create(pXML->pTokenizer->pTokenPool,
    			          &pNode->pData);
      }

      if (err == aErrNone)
        aTokenList_Add(pNode->pData, (aTokenInternal*)pToken);
    }

  } /* bNotClosed */

  return err;

parse_error:
  if (pToken)
    aTokenizerInternal_Dispose(pXML->pTokenizer, pToken);
  if (pKey)
    aTokenizerInternal_Dispose(pXML->pTokenizer, pKey);
  if (pClose)
    aTokenizerInternal_Dispose(pXML->pTokenizer, pClose);

  if (pXML->errorProc) {
    const char* msg[1];
    aTokenInternal* pInt = (aTokenInternal*)pLast;
    msg[0] = pInt->pParseFrame->frameName;
    pXML->errorProc(tkErrXMLMissingClose,
    		    pInt->line, pInt->column, 1, msg,
    		    pXML->errorRef);    		    
  }

  return aErrNone;

} /* sXMLNode_Parse */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sXMLNode_Traverse
 */

aErr sXMLNode_Traverse(aXMLNode* pNode)
{
  aErr err = aErrNone;
  const char* pKey = NULL;

  aAssert(pNode);


  if (pNode->pKeyToken) {
    aAssert(pNode->pKeyToken->eType == tkIdentifier);
    pKey = pNode->pKeyToken->v.identifier;
  }

  /* handle the start before we dig deeper */
  if (pKey && (err == aErrNone) && (pNode->pXML->startProc)) {
    err = pNode->pXML->startProc(pNode, pKey,
    				   pNode->pXML->procRef);
  }

  /* handle the children */
  if (pNode->pChildren) 
    err = sXMLNode_Traverse(pNode->pChildren);

  /* handle the date */
  if (pKey && (err == aErrNone) && pNode->pXML->contentProc
      && pNode->pData) {
    err = sXMLNode_HandleData(pNode, pNode->pXML->contentProc,
    			      pNode->pXML->procRef);
  }

  /* handle the end */
  if (pKey && (err == aErrNone) && (pNode->pXML->endProc)) {
    err = pNode->pXML->endProc(pNode, pNode->pParent, pKey,
    			       pNode->pXML->procRef);
  }

  /* handle any siblings */  
  if ((err == aErrNone) && (pNode->pNext))
    err = sXMLNode_Traverse(pNode->pNext);

  return err;

} /* sXMLNode_Traverse */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sXMLNode_AddChild
 */

aErr sXMLNode_AddChild(aXMLNode* pNode, 
		       aXMLNode* pChild)
{
  aErr err = aErrNone;

  aAssert(pNode);
  
  if ((err == aErrNone) && (pChild != NULL)) {
    if (pNode->pChildren == NULL)
      pNode->pChildren = pChild;
    else {
      aXMLNode* pTemp = pNode->pChildren;
      while (pTemp->pNext != NULL)
        pTemp = pTemp->pNext;
      pTemp->pNext = pChild;
    }
    pChild->pParent = pNode;
  }

  return err;

} /* sXMLNode_AddChild */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sXMLNode_HandleData
 */

aErr sXMLNode_HandleData(aXMLNode* pNode,
			 aXMLHandleContent contentProc,
			 void* vpProcRef)
{
  aErr err = aErrNone;
  const char* pKey = pNode->pKeyToken->v.identifier;

  if ((err == aErrNone) && pKey && contentProc && pNode->pData) {
    aTokenInternal* pTemp = pNode->pData->pHead;
    while (pTemp) {
      contentProc(pNode, pKey, (aToken*)pTemp, vpProcRef);
      pTemp = pTemp->pNext;
    }
  }

  return err;

} /* sXMLNode_HandleData */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aXML_Create
 */

aErr sXML_Parse(aXML* pXML)
{
  aErr err = aErrNone;

  aAssert(pXML);
  aAssert(pXML->data);
  aAssert(pXML->pNodePool);
  aAssert(!pXML->pTree);

  /* first, build a root node to hang things on */
  if (err == aErrNone)
    err = sXMLNode_Create(pXML, NULL, &pXML->pTree);
  
  /* now, do a recursive XML traversal and create the tree */
  if (err == aErrNone) {
    err = sXMLNode_Parse(pXML->pTree);
    if (err == aErrEOF)
      err = aErrNone;
  }

  return err;

} /* sXML_Parse */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aXML_Create
 */

aLIBRETURN aXML_Create(aIOLib ioRef,
		       aStreamRef dataStream,
		       aTokenErrProc errProc,
		       void* errProcRef,
		       aXMLRef* pXMLRef,
		       aErr* pErr)
{
  aErr err = aErrNone;
  aIO* pIO = (aIO*)ioRef;
  aXML* pXML = NULL;

  aVALIDIO(pIO);

  if ((err == aErrNone) && (!pXMLRef || !dataStream))
    err = aErrParam;

  /* try to allocate the structure */
  if (err == aErrNone) {
    pXML = (aXML*)aMemAlloc(sizeof(aXML));
    if (pXML) {
      aBZero(pXML, sizeof(aXML));
      pXML->ioRef = ioRef;
      pXML->data = dataStream;
      pXML->errorProc = errProc;
      pXML->errorRef = errProcRef;
      pXML->check = aXMLCHECK;
    } else 
      err = aErrMemory;
  }
  
  /* build the memory pool for the xml tree nodes */
  if (err == aErrNone) {
    err = aMemPoolInternal_Create(sizeof(aXMLNode),
    				    32,
    				    &pXML->pNodePool);
  }
  
  /* build a tokenizer for the parsing */
  if (err == aErrNone)
    err = aTokenizerInternal_Create(dataStream,
    				    "XML Data",
    				    aFileAreaUser,
    				    pIO->ccMap,
    				    errProc, /* mrw */
    				    errProcRef, /* mrw */
    				    &pXML->pTokenizer);

  /* we are ready to go so parse it */
  if (err == aErrNone)
    err = sXML_Parse(pXML);

  if (err == aErrNone) {
    *pXMLRef = pXML;
  } else {
    aXML_Destroy(ioRef, pXML, NULL);
  }

  if (pErr != NULL)
    *pErr = err;

  return((aLIBRETURN)(err != aErrNone));

} /* aXML_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aXML_Traverse
 */

aLIBRETURN aXML_Traverse(aIOLib ioRef,
			 aXMLRef XML,
			 const aXMLCallbacks* pCallbacks,
			 aErr* pErr)
{
  aErr err = aErrNone;
  aXML* pXML = (aXML*)XML;

  aVALIDIO(ioRef);
  if (err == aErrNone) {
    aVALIDXML(XML);
  }

  if (err == aErrNone) {
  
    /* set up the callback procs */
    pXML->startProc = pCallbacks->handleStart;
    pXML->contentProc = pCallbacks->handleContent;
    pXML->endProc = pCallbacks->handleEnd;
    pXML->procRef = pCallbacks->ref;
    
    /* do the traversal */
    err = sXMLNode_Traverse(pXML->pTree);
 
    /* clear out the procs */
    pXML->startProc = NULL;
    pXML->contentProc = NULL;
    pXML->endProc = NULL;
    pXML->procRef = NULL;
  }

  if (pErr != NULL)
    *pErr = err;

  return((aLIBRETURN)(err != aErrNone));

} /* aXML_Traverse */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aXMLNode_FindKey
 */

aLIBRETURN aXMLNode_FindKey(aIOLib ioRef,
			    aXMLNodeRef XMLNode,
			    const char* pKey,
			    aXMLHandleContent contentProc,
			    void* vpProcRef,
			    aErr* pErr)
{
  aErr err = aErrNone;
  aXMLNode* pNode = (aXMLNode*)XMLNode;
  aXML* pXML = NULL;

  aVALIDIO(ioRef);

  if (err == aErrNone) {
    if (!pNode)
      err = aErrParam;
    else
      pXML = (aXML*)pNode->pXML;
  }
  if (err == aErrNone)
    aVALIDXML(pXML);

  if (err == aErrNone) {
    aXMLNode* pTemp = pNode->pChildren;
    while (pTemp) {
      if (pTemp->pKeyToken && 
      	  !aStringCompare(pTemp->pKeyToken->v.identifier, pKey))
      err = sXMLNode_HandleData(pTemp, contentProc, vpProcRef);
      pTemp = pTemp->pNext;
    }
  }

  if (pErr != NULL)
    *pErr = err;

  return((aLIBRETURN)(err != aErrNone));

} /* aXMLNode_FindKey */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aXML_Destroy
 */

aLIBRETURN aXML_Destroy(aIOLib ioRef,
			aXMLRef XML,
			aErr* pErr)
{
  aErr err = aErrNone;
  aXML* pXML = (aXML*)XML;
  
  aVALIDIO(ioRef);
  if (err == aErrNone) {
    aVALIDXML(XML);
  }

  /* try to clean up the tree */
  if ((err == aErrNone) && pXML->pTree) {
    err = sXMLNode_Destroy(pXML->pTree);
    pXML->pTree = NULL;
  }

  /* clean up the parsing tokenizer */
  if ((err == aErrNone) && pXML->pTokenizer) {
    err = aTokenizerInternal_Destroy(pXML->pTokenizer);
    pXML->pTokenizer = NULL;
  }

  /* clean up the pooled memory allocator */
  if ((err == aErrNone) && pXML->pNodePool) {
    err = aMemPoolInternal_Destroy(pXML->pNodePool);
    pXML->pNodePool = NULL;
  }

  if (pXML)
    aMemFree(pXML);

  if (pErr != NULL)
    *pErr = err;

  return((aLIBRETURN)(err != aErrNone));

} /* aXML_Destroy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aXMLNode_AddContent
 */

aLIBRETURN aXMLNode_AddContent(aIOLib ioRef,
			       aXMLNodeRef XMLNode,
			       const char* pKey,
			       const char* pData,
			       aErr* pErr)
{
  aErr err = aErrNone;

  
  if (pErr != NULL)
    *pErr = err;

  return((aLIBRETURN)(err != aErrNone));

} /* aXMLNode_AddContent */

