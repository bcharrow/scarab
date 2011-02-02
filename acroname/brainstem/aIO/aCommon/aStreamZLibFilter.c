/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStream_CreateZLibFilter.c                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent I/O       */
/*              layer.						   */
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

#include "aStreamZLibFilter.h"

#include "zutil.h"

#define Z_BUFSIZE 16384



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sZLF_Write (
  const char* pData, 
  const unsigned long nSize,
  void* ref
)
{
  aErr err = aErrNone;
  aStreamZLibFilter* pZLF = (aStreamZLibFilter*)ref;
  
  if (pZLF->eMode != aFileModeWriteOnly)
    err = aErrMode;
  
  pZLF->z.next_in = (Bytef*)pData;
  pZLF->z.avail_in = nSize;

  while (pZLF->z.avail_in != 0) {
    if (pZLF->z.avail_out == 0) {
      pZLF->z.next_out = pZLF->buf;
      if (aStream_Write(pZLF->ioRef, 
      		        pZLF->pFilteredStream, 
      		        (char*)pZLF->buf,
      		        Z_BUFSIZE,
      		        &err)) {
        pZLF->z_err = Z_ERRNO;
        break;
      }
      pZLF->z.avail_out = Z_BUFSIZE;
    }

    pZLF->z_err = deflate(&(pZLF->z), Z_NO_FLUSH);

    if (pZLF->z_err != Z_OK) break;
  }

  return err;

} /* sZLF_Write */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sZLF_Put (
  char* pData, void* ref
)
{
  return sZLF_Write(pData, 1L, ref);

} /* sZLF_Put */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sZLF_Get (
  char* pData, 
  void* ref
)
{
  aErr err = aErrNone;
  aStreamZLibFilter* pZLF = (aStreamZLibFilter*)ref;
  Bytef* start = (Bytef*)pData;
  Byte* next_out;

  if (pZLF->eMode != aFileModeReadOnly)
    return aErrMode;

  if (pZLF->z_err == Z_DATA_ERROR || pZLF->z_err == Z_ERRNO)
    return aErrIO;

  if (pZLF->z_err == Z_STREAM_END)
    return aErrEOF;

  if (err == aErrNone) {
    
    next_out = (Byte*)pData;
    pZLF->z.next_out = (Bytef*)pData;
    pZLF->z.avail_out = 1L;

    while (pZLF->z.avail_out != 0) {

      /* read in if buffer is empty */
      if ((pZLF->z.avail_in == 0) && !pZLF->z_eof) {
        int left = Z_BUFSIZE;
        Byte* p = pZLF->buf;
        aAssert(pZLF->pFilteredStream);
        aAssert(pZLF->pFilteredStream->getProc);
        while ((left > 0) && (err == aErrNone)) {
          err = pZLF->pFilteredStream->getProc((char*)p, 
          			pZLF->pFilteredStream->procRef);
          if (err == aErrNone) {
            p++;
            pZLF->z.avail_in++;
            left--;
          } else if (err == aErrEOF) {
            err = aErrNone;
            /* signal end when no more bytes can be read in */
            if (pZLF->z.avail_in == 0)
              pZLF->z_eof = 1;
            break;
          } else if (err == aErrNotReady) {
            return aErrNotReady;
          } else {
            pZLF->z_err = Z_ERRNO;
          }
        }
        pZLF->z.next_in = pZLF->buf;
      }
     
      /* handle the buffer */
      pZLF->z_err = inflate(&(pZLF->z), Z_NO_FLUSH);

      /* reset the stream */
      if (pZLF->z_err == Z_STREAM_END)
        start = pZLF->z.next_out;

      if (pZLF->z_err != Z_OK || pZLF->z_eof) break;
    }
  } /* while avail_out */

  return err;

} /* sZLF_Get */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr sZLF_Delete (
  void* ref
)
{
  aErr err = aErrNone;
  aStreamZLibFilter* pZLF = (aStreamZLibFilter*)ref;

  aVALIDZLF(pZLF);

  if (err == aErrNone) {
    if (pZLF->eMode == aFileModeWriteOnly) {
      uInt len;
      int done = 0;

      /* need to first flush out the output that has accumulated */
      pZLF->z.avail_in = 0;
      for (;;) {
        len = Z_BUFSIZE - pZLF->z.avail_out;
        
        if (len != 0) {
          if (aStream_Write(pZLF->ioRef,
          		    pZLF->pFilteredStream,
          		    (char*)pZLF->buf,
          		    len,
          		    &err)) {
            pZLF->z_err = Z_ERRNO;
            return err;
          }
          pZLF->z.next_out = pZLF->buf;
          pZLF->z.avail_out = Z_BUFSIZE;
        }
        if (done) break;

        pZLF->z_err = deflate(&(pZLF->z), Z_FINISH);

	/* Ignore the second of two consecutive flushes: */
	if ((len == 0) && (pZLF->z_err == Z_BUF_ERROR))
	  pZLF->z_err = Z_OK;

        /* deflate has finished flushing only when it hasn't used up
         * all the available space in the output buffer: 
         */
        done = ((pZLF->z.avail_out != 0) 
        	|| (pZLF->z_err == Z_STREAM_END));

        if ((pZLF->z_err != Z_OK) 
            && (pZLF->z_err != Z_STREAM_END)) break;
      } /* for */

      if (pZLF->z_err == Z_STREAM_END)
        pZLF->z_err = Z_OK;

      if (pZLF->z_err == Z_OK)
        pZLF->z_err = deflateEnd(&(pZLF->z));

    } else {
      pZLF->z_err = inflateEnd(&(pZLF->z));
    }
  }
  
  if (err == aErrNone) {
    if (pZLF->buf) {
      aMemFree(pZLF->buf);
      pZLF->buf = NULL;
    }
  }

  if (err == aErrNone) {
    pZLF->check = 0;
    aMemFree(pZLF);
  }

  return err;

} /* sZLF_Delete */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_CreateZLibFilter
 */

aLIBRETURN aStream_CreateZLibFilter (
  aIOLib ioRef, 
  const aStreamRef streamToFilter,
  const aFileMode eMode,
  aStreamRef* pFilteredStreamRef,
  aErr* pErr
)
{
  aErr err = aErrNone;
  aStreamRef streamRef;
  aStream* pStreamToFilter = (aStream*)streamToFilter;
  aStreamZLibFilter* pZLF = NULL;
  aStreamGetProc pGet = NULL;
  aStreamPutProc pPut = NULL;
  int level = Z_DEFAULT_COMPRESSION; /* compression level */
  int strategy = Z_DEFAULT_STRATEGY; /* compression strategy */

  /* check params */
  if (!ioRef || !pFilteredStreamRef)
    err = aErrParam;
  if ((err == aErrNone) 
      && !((eMode == aFileModeWriteOnly) 
           || (eMode == aFileModeReadOnly)))
    err = aErrParam;
  if (err == aErrNone) {
    aVALIDSTREAM(streamToFilter);
  }
  if (err == aErrNone) {
    if (((eMode == aFileModeReadOnly) && !pStreamToFilter->getProc)
    	|| ((eMode == aFileModeWriteOnly) && !pStreamToFilter->putProc))
      err = aErrMode;
  }

  /* initialize */
  if (err == aErrNone)
    *pFilteredStreamRef = NULL;
    
  if (err == aErrNone) {
    pZLF = (aStreamZLibFilter*)aMemAlloc(sizeof(aStreamZLibFilter));
    if (pZLF) {
      aBZero(pZLF, sizeof(aStreamZLibFilter));
      pZLF->ioRef = ioRef;
      pZLF->eMode = eMode;
      pZLF->check = aZLFCHECK;
      pZLF->pFilteredStream = (aStream*)streamToFilter;
    } else {
      err = aErrMemory;
    }
  }
  
  if (err == aErrNone) {
    pZLF->buf = (Byte*)aMemAlloc(Z_BUFSIZE);
    if (!pZLF->buf)
      err = aErrMemory;
  }

  /* create the actual stream */
  if (err == aErrNone) {

    /* write */
    if (eMode == aFileModeWriteOnly) {

      pPut = sZLF_Put;

      pZLF->z_err = deflateInit2(&(pZLF->z), 
      			  level,
                          Z_DEFLATED, 
                          MAX_WBITS, 
                          DEF_MEM_LEVEL, 
                          strategy);

      if (pZLF->z_err == Z_OK)
        pZLF->z.next_out = pZLF->buf;
      else 
        err = aErrIO;

    /* read */
    } else {

      pGet = sZLF_Get;

      pZLF->z.next_in  = pZLF->buf;
      pZLF->z_err = inflateInit2(&(pZLF->z), MAX_WBITS);
      if (pZLF->z_err != Z_OK)
        err = aErrIO;
    }

    if (err == aErrNone)
      pZLF->z.avail_out = Z_BUFSIZE;
  }

  if (err == aErrNone)
    aStream_Create(ioRef,
    		   pGet,
    		   pPut,
    		   sZLF_Delete,
    		   pZLF,
    		   &streamRef,
    		   &err);

  if (err == aErrNone) {
    /* setting sStreamBufferWrite allows bulk writes without
     * calling the put proc once for each byte.  This is much
     * more efficient.
     */
    aStream* pStream = (aStream*)streamRef;
    pStream->writeProc = sZLF_Write;
    *pFilteredStreamRef = streamRef;
  } else {
    if (pZLF)
      aMemFree(pZLF);
  }
  
  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStream_CreateZLibFilter */
