/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aToken.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: implementation of an efficient memory allocator    */
/*              for identical sized blocks. 	                   */
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

#include "aIOInternal.h"
#include "aTokenInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSymbolTable_Create
 */

aLIBRETURN aTokenizer_Create(aIOLib ioRef,
			     aStreamRef tokenStream,
			     const char* streamName,
		       	     aFileArea eIncludeArea,
		       	     aTokenErrProc errProc,
		       	     void* errProcRef,
			     aTokenizerRef* pTokenizerRef,
		     	     aErr* pErr)
{
  aErr err = aErrNone;
  aIO* pIO = (aIO*)ioRef;

  aVALIDIO(pIO);

  if (err == aErrNone)
    err = aTokenizerInternal_Create(tokenStream, 
    				    streamName,
  			            eIncludeArea, 
  			            pIO->ccMap,
  			            errProc,
  			            errProcRef,
  			            (aTokenizer**)pTokenizerRef);
  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aTokenizer_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenizer_Next
 */

aLIBRETURN aTokenizer_Next(aIOLib ioRef,
			   aTokenizerRef tokenizerRef,
			   aToken** ppToken,
		     	   aErr* pErr)
{
  aErr err = aTokenizerInternal_Next((aTokenizer*)tokenizerRef,
  				       ppToken);
  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aTokenizer_Next */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aToken_GetInfo
 */

aLIBRETURN aToken_GetInfo(aIOLib ioRef,
			  const aToken* pToken,
			  aTokenInfo* pTokenInfo,
		     	  aErr* pErr)
{
  aErr err = aTokenInternal_GetInfo(pToken,
  				    pTokenInfo);
  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aToken_GetInfo */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenizer_PushBack
 */

aLIBRETURN aTokenizer_PushBack(aIOLib ioRef,
			       aTokenizerRef tokenizerRef,
			       aToken* pToken,
		     	       aErr* pErr)
{
  aErr err = aTokenizerInternal_PushBack((aTokenizer*)tokenizerRef,
  				           pToken);
  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aTokenizer_PushBack */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenizer_Dispose
 */

aLIBRETURN aTokenizer_Dispose(aIOLib ioRef,
			      aTokenizerRef tokenizerRef,
			      aToken* pToken,
		     	      aErr* pErr)
{
  aErr err = aTokenizerInternal_Dispose((aTokenizer*)tokenizerRef,
  				        pToken);
  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aTokenizer_Dispose */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenizer_Destroy
 */

aLIBRETURN aTokenizer_Destroy(aIOLib ioRef,
			      aTokenizerRef tokenizerRef,
		     	      aErr* pErr)
{
  aErr err = aTokenizerInternal_Destroy((aTokenizer*)tokenizerRef);

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aTokenizer_Destroy */

