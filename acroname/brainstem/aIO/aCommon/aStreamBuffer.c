/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStreamBuffer.c                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: cross-platform data buffer stream definition.      */
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
#include "aStream.h"
#include "aStreamBuffer.h"

#define aSBD_BYTESTOEND(pSBD) (pSBD->nBufferSize - pSBD->nStart)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sStreamBufferGet(char* pData, void* ref)
{
  aErr err = aErrNone;
  aStreamBufferData* pSBD = (aStreamBufferData*)ref;

  aVALIDSBD(pSBD);
  if ((err == aErrNone) && !pData)
    err = aErrParam;

  if (err == aErrNone) {
    if (pSBD->nBytes > 0) {
      *pData = pSBD->pBuffer[pSBD->nStart++];
      /* wrap if we went off the end */
      if (pSBD->nStart == pSBD->nBufferSize)
        pSBD->nStart = 0;
      pSBD->nBytes--;
    } else
      err = aErrEOF;
  }

  return err;

} /* sStreamBufferGet */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sStreamBufferPut(char* pData, void* ref)
{
  aErr err = aErrNone;
  aStreamBufferData* pSBD = (aStreamBufferData*)ref;

  aVALIDSBD(pSBD);
  if ((err == aErrNone) && !pData)
    err = aErrParam;

  /* grow the buffer if needed */
  if ((err == aErrNone) 
      && ((pSBD->nBytes + 1) > pSBD->nBufferSize)) {
    err = sEnsureContinuous(pSBD);
    if (err == aErrNone) {
    aMemSize newSize = pSBD->nBufferSize + pSBD->nIncSize;
    char* pNewBuffer = (char*)aMemAlloc(newSize);
    if (pNewBuffer) {
      if (pSBD->pBuffer) {
        aMemCopy(pNewBuffer, pSBD->pBuffer, pSBD->nBufferSize);
        aMemFree(pSBD->pBuffer);
      }
      pSBD->pBuffer = pNewBuffer;
      pSBD->nBufferSize = newSize;
    } else
      err = aErrMemory;
  }
  }

  /* compute the offset for the new buffer */
  if (err == aErrNone) {
    aMemSize o = (pSBD->nStart + pSBD->nBytes) % pSBD->nBufferSize;
    pSBD->pBuffer[o] = *pData;
    pSBD->nBytes++;
  }

  return err;

} /* sStreamBufferPut */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sStreamBufferWrite(const char* pData, 
			const unsigned long nSize,
			void* ref)
{
  aErr err = aErrNone;
  aStreamBufferData* pSBD = (aStreamBufferData*)ref;
  aMemSize s;

  aVALIDSBD(pSBD);
  if ((err == aErrNone) && !pData)
    err = aErrParam; 
 
  /* make the buffer zero based when empty */
  if ((err == aErrNone) && (pSBD->nBytes == 0))
    pSBD->nStart = 0;

  /* grow the buffer if necessary */
  if (err == aErrNone) {

    /* do we need to grow the buffer? */
    if ((pSBD->nBytes + nSize) > pSBD->nBufferSize) {

      /* while we are growing, add some extra for future growth */
      aMemSize newSize = pSBD->nBufferSize + nSize + pSBD->nIncSize;
      char* pNewBuffer = (char*)aMemAlloc(newSize);
      if (pNewBuffer) {
        if (pSBD->pBuffer) {

          /* ensure a zero based buffer results when copying */
          if (pSBD->nStart + pSBD->nBytes <= pSBD->nBufferSize) {
            /* .....SxxxxxxxxE..... */
            /* SxE */
            /* SxxxxxxxxxxxxxxxxxxE */
            aMemCopy(pNewBuffer, &pSBD->pBuffer[pSBD->nStart], 
            	     pSBD->nBytes);
          } else {
            /* xxxxE..........Sxxxx */
            /* E.S */
            /* E..................S */
            s = aSBD_BYTESTOEND(pSBD);
            aMemCopy(pNewBuffer, &pSBD->pBuffer[pSBD->nStart], s);
            aMemCopy(&pNewBuffer[s], pSBD->pBuffer, 
            	     (aMemSize)(pSBD->nBytes - s));
          }
          aMemFree(pSBD->pBuffer);
        }
        pSBD->nStart = 0;
        pSBD->pBuffer = pNewBuffer;
        pSBD->nBufferSize = newSize;
        /* finally, add in the new write data */
        aMemCopy(&pSBD->pBuffer[pSBD->nBytes], pData, nSize);
        pSBD->nBytes += nSize;
      } else
        err = aErrMemory;

    /* else, there is enough buffer space */
    } else {
      aMemSize e = (pSBD->nStart + pSBD->nBytes) % pSBD->nBufferSize;

            /* .....SxxxxxxxxE..... 
             * xxE..Sxxxxxxxxxxxxxx */

      /* compute and perform the first write */
      if (e + nSize < pSBD->nBufferSize)
        s = nSize; /* all the bytes can be written */
      else
        s = pSBD->nBufferSize - e;
      aMemCopy(&pSBD->pBuffer[e], pData, s);
      e = nSize - s;

      /* do the second write if needed */
      if (e)
        aMemCopy(pSBD->pBuffer, &pData[s], e);
      
      /* account for the write */
      pSBD->nBytes += nSize;
    }
  }

  return err;

} /* sStreamBufferWrite */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sStreamBufferDelete(void* ref)
{
  aErr err = aErrNone;
  aStreamBufferData* pSBD = (aStreamBufferData*)ref;

  aVALIDSBD(pSBD);
  
  if ((err == aErrNone) && (pSBD->pBuffer)) {
    aMemFree(pSBD->pBuffer);
    pSBD->pBuffer = NULL;
  }
  
  if (err == aErrNone) {
    pSBD->check = 0;
    aMemFree(pSBD);
  }

  return err;

} /* sStreamBufferDelete */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sEnsureContinuous(aStreamBufferData* pSBD)
{
  aErr err = aErrNone;
  
  aVALIDSBD(pSBD);

  /* if we need to, copy the buffer into a continiguous block */
  if ((err == aErrNone) 
      && (pSBD->nStart + pSBD->nBytes > pSBD->nBufferSize)) {
    char* pNewBuffer;
    aMemSize s, r;

    /* xxxxE..........Sxxxx */
    /* E.S */
    /* E..................S */

    pNewBuffer = (char*)aMemAlloc(pSBD->nBufferSize);
    if (pNewBuffer) {
      s = aSBD_BYTESTOEND(pSBD);
      aMemCopy(pNewBuffer, &pSBD->pBuffer[pSBD->nStart], s);
      r = pSBD->nBytes - s;
      aMemCopy(&pNewBuffer[s], pSBD->pBuffer, r);
      aMemFree(pSBD->pBuffer);
      pSBD->pBuffer = pNewBuffer;
      pSBD->nStart = 0;
    } else
      err = aErrMemory;
  }

  return err;

} /* sEnsureContinuous */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aLIBRETURN aStreamBuffer_Create(aIOLib ioRef, 
			        const aMemSize nIncSize,
			        aStreamRef* pBufferStreamRef,
			        aErr* pErr)
{
  aErr err = aErrNone;
  aStreamRef streamRef;
  aStreamBufferData* pSBD = NULL;


  /* check params */
  if (!ioRef || !pBufferStreamRef)
    err = aErrParam;
  else if (nIncSize < 1)
    err = aErrRange;

  /* initialize */
  if (err == aErrNone)
    *pBufferStreamRef = NULL;
    
  if (err == aErrNone) {
    pSBD = (aStreamBufferData*)aMemAlloc(sizeof(aStreamBufferData));
    if (pSBD) {
      aBZero(pSBD, sizeof(aStreamBufferData));
      pSBD->ioRef = ioRef;
      pSBD->nIncSize = nIncSize;
      pSBD->check = aSBDCHECK;
    } else {
      err = aErrMemory;
    }
  }

  /* create the actual stream */
  if (err == aErrNone)
    aStream_Create(ioRef,
    		   sStreamBufferGet,
    		   sStreamBufferPut,
    		   sStreamBufferDelete,
    		   pSBD,
    		   &streamRef,
    		   &err);

  if (err == aErrNone) {
    /* setting sStreamBufferWrite allows bulk writes without
     * calling the put proc once for each byte.  This is much
     * more efficient.
     */
    aStream* pStream = (aStream*)streamRef;
    pStream->writeProc = sStreamBufferWrite;
    *pBufferStreamRef = streamRef;
  } else {
    if (pSBD)
      aMemFree(pSBD);
  }
  
  if (pErr)
    *pErr = err;
  
  return (aLIBRETURN)(err != aErrNone);

} /* aStreamBuffer_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aLIBRETURN aStreamBuffer_Get(aIOLib ioRef,
			     aStreamRef bufferStreamRef,
		             aMemSize* pSize,
		             char** ppData,
		             aErr* pErr)
{
  aErr err = aErrNone;
  aStream* pStream = (aStream*)bufferStreamRef;
  aStreamBufferData* pSBD = NULL;

  /* make sure we were handed the right stuff */
#ifndef aPALM
  /* for palm, this will fail since we have a different 
   * mechanism for libraries. Since we don't every actually 
   * use the ioRef in this call, it is ok to ignore this 
   * paremeter in this specific case.
   */
  aVALIDIO(ioRef);
#endif
  if (err == aErrNone)
    aVALIDSTREAM(pStream);
  if (err == aErrNone) {
    pSBD = (aStreamBufferData*)pStream->procRef;
    aVALIDSBD(pSBD);
  }

  /* get the size if requested */
  if ((err == aErrNone) && pSize)
    *pSize = pSBD->nBytes;

  if ((err == aErrNone) && ppData) {
    err = sEnsureContinuous(pSBD);
    if (err == aErrNone)
      *ppData = (pSBD->pBuffer) ? &pSBD->pBuffer[pSBD->nStart] : 
      				  NULL;
  }

  if (pErr)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStreamBuffer_Get */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStreamBuffer_Flush
 */

aLIBRETURN aStreamBuffer_Flush(
  aIOLib ioRef,
  aStreamRef bufferStreamRef,
  aStreamRef flushStream,
  aErr* pErr)
{
  aErr err = aErrNone;
  aStream* pStream = (aStream*)bufferStreamRef;
  aStream* pFlush = (aStream*)flushStream;
  aStreamBufferData* pSBD = NULL;
    
  /* make sure we were handed the right stuff */
#ifndef aPALM
  /* for palm, this will fail since we have a different 
   * mechanism for libraries. Since we don't every actually 
   * use the ioRef in this call, it is ok to ignore this 
   * paremeter in this specific case.
   */
  aVALIDIO(ioRef);
#endif
  if (err == aErrNone)
    aVALIDSTREAM(pStream);
  if (err == aErrNone) {
    pSBD = (aStreamBufferData*)pStream->procRef;
    aVALIDSBD(pSBD);
  }

  /* make sure we are able to read from the buffer and
   * put or write to the flush stream */
  if (err == aErrNone) {
    if (pStream->getProc) {
      if (pFlush && !pFlush->putProc && !pFlush->writeProc)
        err = aErrMode;
    } else
      err = aErrMode;
  }

  /* do block transfer or byte-by-byte transfer */
  /* based on existence of write proc */
  if (err == aErrNone) {

    unsigned long i;
    unsigned long ulLength;
    char *p;
    
    ulLength = pSBD->nBytes;

    if (pFlush) {
      if ((pFlush->writeProc != NULL) && (ulLength > 1))
        err = pFlush->writeProc(pSBD->pBuffer, ulLength, pFlush->procRef);
      else {
        p = (char *)pSBD->pBuffer;
        for (i = 0; (err == aErrNone) && (i < ulLength); i++, p++) 
          err = pFlush->putProc(p, pFlush->procRef);
      }
    }
    if (err == aErrNone) {
      pSBD->nStart = 0;
      pSBD->nBytes = 0;
    }
  }

  if (pErr)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStreamBuffer_Flush */
