/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aHTTP.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Cross-Platform implementation of basic drawing     */
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

#include "aUIInternal.h"
#include "aUtil.h"
#include "aHTTP.h"
#include "acpString.h"

#ifdef aMACX
#include <time.h>
#include <sys/time.h>
#endif /* aMACX */



/* local functions */

aErr sMemFlush(aIOLib ioRef, aStreamRef buffer, aStreamRef port);

char ctohex(char* pc);


/* this routine does a single write flush which should be far
 * fewer packets than individual writes in the standard flush */
aErr sMemFlush(aIOLib ioRef, aStreamRef buffer, aStreamRef port)
{
  aErr e = aErrNone;
  aMemSize size;
  char* pData;

  /* find the buffer data and size */
  if (e == aErrNone)
    aStreamBuffer_Get(ioRef, buffer, &size, &pData, &e);
  
  if (e == aErrNone)
    aStream_Write(ioRef, port, pData, size, &e);
  
  if (e == aErrNone)
    aStream_Flush(ioRef, buffer, NULL, &e);
  
  return e;
}


char ctohex(char* pc)
{
  if ((*pc >= '0') && (*pc <= '9')) return (char)((*pc) - '0');
  if ((*pc >= 'A') && (*pc <= 'F')) return (char)(((*pc) - 'A') + 10);
  if ((*pc >= 'a') && (*pc <= 'f')) return (char)(((*pc) - 'a') + 10);
  return 0;
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHTTP_LogLine
 */

aErr sHTTP_LogLine(aHTTP* pHTTP, 
		   const char* line)
{
  aErr uiErr = aErrNone;

/* #ifdef aDEBUG */
  aVALIDHTTP(pHTTP);

  if ((uiErr == aErrNone) && pHTTP->log) {
    unsigned long time;
    acpString timestamp;
    
    aAssert(pHTTP->ioRef);

    aIO_GetMSTicks(pHTTP->ioRef, &time, &uiErr);
    timestamp = time;
    timestamp += ":";
    timestamp += pHTTP->nCur;
    timestamp += ": ";
    if (!aStream_Write(pHTTP->ioRef, pHTTP->log, 
    		       timestamp, timestamp.length(),
    		       &uiErr))
      aStream_WriteLine(pHTTP->ioRef, pHTTP->log, line, &uiErr);
  }
/* #endif */

  return uiErr;
  
} /* sHTTP_LogLine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHTTP_HandleRequest
 */

aErr sHTTP_HandleRequest(
  aHTTP* pHTTP, 
  const char* line
)
{
  aErr uiErr = aErrNone;
  aBool bHeaderDone = aFalse;
  char input[aHTTPMAXREQESTLINE];
  int contentLength = 0;
  const char* rest;
  const char* asset = (char*)NULL;
  char url[aHTTPMAXREQESTLINE];
//  char* pParams = (char*)NULL;
  acpString allParams;
  aSymbolTableRef paramSyms = NULL;
  aBool bCloseConnection = aFalse;
  acpString mimeType;
  aMemSize replySize;
 
  aVALIDHTTP(pHTTP);

  /* filter out blank input lines */
  if ((uiErr == aErrNone) && (line[0] == 0)) {
    uiErr = sHTTP_LogLine(pHTTP, "filtered blank line");
    return uiErr;
  }

  aAssert(uiErr == aErrNone);

  /* log the request */
  if (uiErr == aErrNone)
    uiErr = sHTTP_LogLine(pHTTP, line);
  	
  aAssert(uiErr == aErrNone);


  /* figure out the first line of the request */
  if (uiErr == aErrNone) {
    if ((rest = aString_StartsWith(line, "GET "))) {
      char* p = (char*)rest;
      char* v = (char*)aString_CopyToWS(url, 
				        aHTTPMAXREQESTLINE, rest);
      
      /* find the HTTP version */
      while (*v && ((*v == ' ') || (*v == '\t')))
        v++;
      /* close the connection for backwards compatibility */
      if (!aStringCompare(v, "HTTP/1.0"))
        bCloseConnection = aTrue;

      /* search for the ? indicating parameters */
      while (*p && *p != '?') p++;
      
      /* terminate the url here, then get parameters */
      if (*p == '?') {
        url[p - rest] = 0;
        *p++ = 0;
      }

      /* copy up to whitespace for the params */
      while ((*p != ' ') && (*p != '\t'))
	allParams += *p++;

    } else if ((rest = aString_StartsWith(line, "POST "))) {
      aString_CopyToWS(url, aHTTPMAXREQESTLINE, rest);
    } else {
      *url = 0;
    }
  }

  aAssert(uiErr == aErrNone);

  /* url now contains the file request with parameters stripped */

  /* gobble up lines until we get a blank one establishing the
   * request settings as we go */
  while ((uiErr == aErrNone) && !bHeaderDone) {
  
    /* get the next line of the header */
    aStream_ReadLine(pHTTP->ioRef, pHTTP->ports[pHTTP->nCur], 
    		     input, aHTTPMAXREQESTLINE, &uiErr);

    /* we may be outpacing the TCP/IP transport so it is ok 
     * if the stream is not ready */
    if (uiErr == aErrNotReady)
      uiErr = aErrNone;

    /* blank lines signal the end of the header */
    if ((uiErr == aErrNone) && (*input == 0))
      bHeaderDone = aTrue;

    if (!bHeaderDone) {
      /* show the header line */
      uiErr = sHTTP_LogLine(pHTTP, input);
     
      /* now, try to get information from the header lines we
       * care about */
      if ((rest = aString_StartsWith(input, "Content-Length:"))
          || (rest = aString_StartsWith(input, "Content-length:"))) {
        aIntFromString(&contentLength, rest);
      } else if ((rest = aString_StartsWith(input, "Connection:"))
	         && !bCloseConnection) {
        /* skip over whitespace */
        while (*rest == ' ') rest++;
        if (!aStringCompare(rest, "close"))
          bCloseConnection = aTrue;
        else if (!aStringCompare(rest, "Keep-Alive"))
          bCloseConnection = aFalse;
      }
    }
  }

  aAssert(uiErr == aErrNone);
  
  /* if we have contentLength, go get the POST parameters 
   * and add them to any get parameters that were set */
  if ((uiErr == aErrNone) && contentLength) {

    /* slurp in the parameters */
    char* pParams = (char*)aMemAlloc((aMemSize)(contentLength + 1));
    if (!pParams)
      uiErr = aErrMemory;
    else {
      aErr readErr;
      aStream_Read(pHTTP->ioRef, pHTTP->ports[pHTTP->nCur],
      		   pParams, (unsigned long)contentLength, 
      		   &readErr);
      /* what if the post data is still coming in? This should perhaps
       * be a blocking read with timeout */
      aAssert(readErr == aErrNone);
      
      /* here, the client may send a CR/LF so we might want to check */

      if (uiErr == aErrNone)
        pParams[contentLength] = 0;
      if (!allParams.length())
        allParams += '&';
      allParams += pParams;

      aMemFree(pParams);
    }
  }

  aAssert(uiErr == aErrNone);

  /* show the parameters all glued together */
  if (uiErr == aErrNone) {
    acpString msg("  params = ");
    msg += allParams;
    uiErr = sHTTP_LogLine(pHTTP, msg);
  }

  aAssert(uiErr == aErrNone);

  /* build up the parameter symbol table */
  if ((uiErr == aErrNone) && allParams.length()) {
    uiErr = sHTTP_ParseParams(pHTTP, allParams, &paramSyms);
  }

  aAssert(uiErr == aErrNone);
  
  sHTTP_LogLine(pHTTP, "params parsed");

  /* set up the mime type and establish if it is a dynamic
   * or static request */
  if ((uiErr == aErrNone)
      && !aUtil_GetFileExtension(input, url, &uiErr)) { 
    if (!aStringCompare(input, ".jpg"))
      mimeType = "image/jpeg";
    else if (!aStringCompare(input, ".gif"))
      mimeType = "image/gif";
    else if (!aStringCompare(input, ".mpg"))
      mimeType = "image/mpeg";
    else if (!aStringCompare(input, ".png"))
      mimeType = "image/png";
    else if (!aStringCompare(input, ".css"))
      mimeType = "text/css";
    else
      mimeType = "text/html";

    if ((asset = aString_StartsWith(url, "/aAsset/")))
      *url = 0;
  }

  aAssert(uiErr == aErrNone);

  /* clear out any existing header buffer */
  if (uiErr == aErrNone)
    aStream_Flush(pHTTP->ioRef, pHTTP->headerBuffer, 
    			NULL, &uiErr);

  aAssert(uiErr == aErrNone);

  /* clear out any existing reply buffer */
  if (uiErr == aErrNone)
    aStream_Flush(pHTTP->ioRef, pHTTP->replyBuffer, 
    			NULL, &uiErr);

  aAssert(uiErr == aErrNone);

  /* fill up the reply from either the assets or callbacks */
  if (uiErr == aErrNone) {
    if (asset)
      uiErr = sHTTP_HandleAsset(pHTTP, asset, 
      				pHTTP->replyBuffer);
    else {
      uiErr = pHTTP->requestProc(url, paramSyms, 
	    		         pHTTP->replyBuffer, 
	    		         pHTTP->vpRef);
    }

    /* write the appropriate response header based on the error */
    switch (uiErr) {

    case aErrNone:
      aStream_WriteLine(pHTTP->ioRef, pHTTP->headerBuffer, 
      			"HTTP/1.1 200 OK", &uiErr);
      break;

    case aErrNotFound:
      aStream_WriteLine(pHTTP->ioRef, pHTTP->headerBuffer, 
      			"HTTP/1.1 200 OK", &uiErr);
      if (uiErr == aErrNone)
        uiErr = sHTTP_DisplayMsg(pHTTP, pHTTP->replyBuffer, "Not Found");
      break;

    default:
      aStream_WriteLine(pHTTP->ioRef, pHTTP->headerBuffer, 
      			"HTTP/1.1 200 OK", &uiErr);
      if (uiErr == aErrNone)
        uiErr = sHTTP_DisplayMsg(pHTTP, pHTTP->replyBuffer, "Not Found Default");
      break;

    } /* switch */
  }

  aAssert(uiErr == aErrNone);

  /* build up and send the header as one write call */
  if (uiErr == aErrNone) {
    aStream_WriteLine(pHTTP->ioRef, pHTTP->headerBuffer, 
    		      "Server: Acroname aHTTP", &uiErr);

    acpString param("Connection: ");
    if (bCloseConnection)
      param += "close";
    else
      param += "Keep-Alive";
    aStream_WriteLine(pHTTP->ioRef, pHTTP->headerBuffer, 
    		      param, &uiErr);
 
    param = "Content-Type: ";
    param += mimeType;
    aStream_WriteLine(pHTTP->ioRef, pHTTP->headerBuffer, 
    		      param, &uiErr);

    sHTTP_LogLine(pHTTP, "header started");

#ifdef aEXTRAMESSAGE
    {
      char date[100];
      struct timeval t;
      gettimeofday(&t, NULL);
      strftime(date, 100, "%a, %d %h %Y %H:%M:%S %Z",
               gmtime((time_t*)&t.tv_sec));
      aStringCopy(param, "Date: ");
      aStringCat(param, date);
      sHTTP_LogLine(pHTTP, param);
      aStream_WriteLine(pHTTP->ioRef, pHTTP->headerBuffer, 
    		      param, &uiErr);
    }
#endif
  }

  aAssert(uiErr == aErrNone);

  /* find the size of the reply */
  if (uiErr == aErrNone)
    aStreamBuffer_Get(pHTTP->ioRef, pHTTP->replyBuffer,
    		      &replySize, (char**)NULL, &uiErr);

  aAssert(uiErr == aErrNone);

  /* the entire reply is now buffered so finish and dump the header */
  if (uiErr == aErrNone) {
    acpString temp("Content-Length: ");
    temp += (int)replySize;
    aStream_WriteLine(pHTTP->ioRef, pHTTP->headerBuffer, 
    		      temp, &uiErr);

    aAssert(uiErr == aErrNone);

    sHTTP_LogLine(pHTTP, "header dumped");

#ifdef aDEBUG
    sHTTP_LogLine(pHTTP, temp);
#endif /* aDEBUG */

    /* add header terminator */
    aStream_WriteLine(pHTTP->ioRef, pHTTP->headerBuffer, 
    		      "", &uiErr);

    aAssert(uiErr == aErrNone);

    /* flush the header buffer to the reply stream */
    uiErr = sMemFlush(pHTTP->ioRef, 
                      pHTTP->headerBuffer,
    		      pHTTP->ports[pHTTP->nCur]);
    if (uiErr != aErrNone)
      uiErr = sHTTP_RebuildStream(pHTTP, pHTTP->nCur);
  }

  aAssert(uiErr == aErrNone);

  /* then write out the reply */
  if (uiErr == aErrNone) {
    uiErr = sMemFlush(pHTTP->ioRef, pHTTP->replyBuffer,
    		      pHTTP->ports[pHTTP->nCur]);
    if (uiErr != aErrNone)
      uiErr = sHTTP_RebuildStream(pHTTP, pHTTP->nCur);
  }

  aAssert(uiErr == aErrNone);

  /* when there is a connection error, the client likely closed down
   * the socket while or before we were writing to it.  Clean this 
   * error out and force a socket rebuild.
   */
  if (uiErr == aErrConnection) {
    uiErr = aErrNone;
    /*    bCloseConnection = aTrue; */
  }

  if ((uiErr == aErrNone) && bCloseConnection) {
    aIO_MSSleep(pHTTP->ioRef, 100, (aErr*)NULL);
    uiErr = sHTTP_RebuildStream(pHTTP, pHTTP->nCur);
  }

  aAssert(uiErr == aErrNone);

  if (paramSyms)
    aSymbolTable_Destroy(pHTTP->ioRef, paramSyms, (aErr*)NULL);

  aAssert(uiErr == aErrNone);

  if (uiErr != aErrNone)
    uiErr = aErrUnknown;

  return uiErr;
  
} /* sHTTP_HandleRequest */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHTTP_RebuildStream
 */

aErr sHTTP_RebuildStream(aHTTP* pHTTP, const int i)
{
  aErr uiErr = aErrNone;

  aVALIDHTTP(pHTTP);

  if ((uiErr == aErrNone) && pHTTP->ports[i]) {
    aStream_Destroy(pHTTP->ioRef, pHTTP->ports[i], &uiErr);
    pHTTP->ports[i] = NULL;
  }

  if (uiErr == aErrNone) {
    acpString line;
    line.format("rebuilding port %d", i);
    uiErr = sHTTP_LogLine(pHTTP, line);
  }

  if (uiErr == aErrNone)
    aStream_CreateSocket(pHTTP->ioRef, 
    			 pHTTP->nInetAddr, 
    			 pHTTP->nPort, 
    			 aTrue,
    			 &pHTTP->ports[i],
    			 &uiErr);

  aAssert(uiErr == aErrNone);

  return uiErr;
  
} /* sHTTP_RebuildStream */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHTTP_HandleAsset
 */

aErr sHTTP_HandleAsset(
  aHTTP* pHTTP, 
  const char* name,
  aStreamRef replyStream
)
{
  aErr uiErr = aErrNone;
  unsigned long size;
  unsigned long* pCache = (unsigned long*)NULL;

  aVALIDHTTP(pHTTP);
  
  aAssert(pHTTP->assetCache);

  sHTTP_LogLine(pHTTP, "in asset");

  /* seek the asset in the cache */
  if (uiErr == aErrNone) {
    void* pVoid;
    aSymbolTable_Find(pHTTP->ioRef,
    		      pHTTP->assetCache,
    		      name,
    		      &pVoid, 
    		      &uiErr);
    if (uiErr == aErrNotFound) {
      pCache = (unsigned long*)NULL;
      uiErr = aErrNone;
    } else {
      pCache = (unsigned long*)pVoid;
      size = *pCache++;
    }
  }

  sHTTP_LogLine(pHTTP, "cache searched");

  /* if not cached, read in and cache */
  if ((uiErr == aErrNone) && !pCache) {
    unsigned long* pNewCache = NULL;
    aFileRef file = NULL;

    sHTTP_LogLine(pHTTP, "loading");
    
    /* open the file */
    aFile_Open(pHTTP->ioRef, name, aFileModeReadOnly,
      	       aFileAreaAsset, &file, &uiErr);

    sHTTP_LogLine(pHTTP, "getting size");

    /* find the file's size for a block read */
    if (uiErr == aErrNone)
      aFile_GetSize(pHTTP->ioRef, file, &size, &uiErr);

    sHTTP_LogLine(pHTTP, "copying into memory");

    /* now build a copy for the cache with the size up front */
    pCache = (unsigned long*)aMemAlloc(size + sizeof(unsigned long));

    if (pCache) {
      sHTTP_LogLine(pHTTP, "malloced");
      pNewCache = pCache;
      *pCache++ = size;
      aFile_Read(pHTTP->ioRef, (aFileRef)file, (char*)pCache, size, 
      		 (unsigned long*)NULL, &uiErr);
    } else 
      uiErr = aErrMemory;

    sHTTP_LogLine(pHTTP, "inserting into cache");

    /* add the copy to the cache */
    if (uiErr == aErrNone)
      aSymbolTable_Insert(pHTTP->ioRef,
      		          pHTTP->assetCache,
      		          name,
      		          pNewCache,
      		          sHTTP_FreeAssetCache,
      		          pHTTP,
      		          &uiErr);

    /* close the file no matter what happened */
    if (file)
      aFile_Close(pHTTP->ioRef, file, (aErr*)NULL); 
  }

  sHTTP_LogLine(pHTTP, "writing reply");

  /* get the reply from the cache */
  if ((uiErr == aErrNone) && pCache)
    aStream_Write(pHTTP->ioRef, replyStream, (char*)pCache, 
    		  size, &uiErr);

  sHTTP_LogLine(pHTTP, "done with asset");

  return uiErr;

} /* sHTTP_HandleAsset */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHTTP_ParseParams
 */

aErr sHTTP_ParseParams(aHTTP* pHTTP, 
		       const char* paramStr, 
		       aSymbolTableRef* pSymRef)
{
  aErr uiErr = aErrNone;
  aSymbolTableRef symbols = NULL;
  const char* ps = paramStr;
  char key[aHTTPMAXPARAMLEN];
  char* pParam = NULL;
  char* pc;

  while (*ps && (uiErr == aErrNone)) {
    void* pVoid;

    if (uiErr == aErrNone)
      aMemPool_Alloc(pHTTP->ioRef, pHTTP->paramPool, 
      		     &pVoid, &uiErr);

    if (uiErr == aErrNone) {
      pParam = (char*)pVoid;
      ps = aString_CopyToChar(key, ps, '=');
      ps = aString_CopyToChar(pParam, ps, '&');
      if (!symbols)
        aSymbolTable_Create(pHTTP->ioRef, 
      			  &symbols,
      			  &uiErr);
    }

    /* post-process any weird text */
    /* HTML replaces spaces with + signs */
    /* HTML converts symbols to escaped hex values (%XX) */
    /* pParam string will only get smaller */
    
    /* first convert any + signs to spaces */
    pc = pParam;
    while (*pc) {
      if (*pc == '+') *pc = ' ';
      pc++;
    }
    
    /* crunch any %XX escape sequences */
    pc = pParam;
    while (*pc) {
      if (*pc == '%') {
        *pc = (char)(ctohex(pc + 1) * 16 + ctohex(pc + 2));
	const char* s = pc + 3;
	char* d = pc + 1;
	while (*s) *d++ = *s++;
	*d++ = 0;
      }
      pc++;
    }

    if (uiErr == aErrNone) {
      aSymbolTable_Insert(pHTTP->ioRef,
      			  symbols,
      			  key,
      			  pParam,
      			  sHTTP_FreeParamVal,
      			  pHTTP,
      			  &uiErr);
      			  
    }
  }
  
  if (uiErr == aErrNone)
    *pSymRef = symbols;

  return uiErr;

} /* sHTTP_ParseParams */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHTTP_FreeParamVal
 */

aErr sHTTP_FreeParamVal(void* pData, void* ref)
{
  aErr uiErr = aErrNone;
  aHTTP* pHTTP = (aHTTP*)ref;

  aVALIDHTTP(pHTTP);
  
  aAssert(pHTTP->paramPool);

  if (uiErr == aErrNone)
    aMemPool_Free(pHTTP->ioRef, pHTTP->paramPool, pData, &uiErr);

  return uiErr;

} /* sHTTP_FreeParamVal */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHTTP_FreeAssetCache
 */

aErr sHTTP_FreeAssetCache(void* pData, void* ref)
{
  aErr uiErr = aErrNone;
  aHTTP* pHTTP = (aHTTP*)ref;

  aVALIDHTTP(pHTTP);

  aAssert(uiErr == aErrNone);

  if (uiErr == aErrNone)
    aMemFree(pData);

  return uiErr;

} /* sHTTP_FreeAssetCache */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHTTP_WriteTemplateBuffer
 * 
 * This handles the text between <BLOCK></BLOCK> tags doing 
 * substitutions where specified.  If any substitution proc returns 
 * an error the entire block is not written.
 */

aErr sHTTP_WriteTemplateBuffer(aHTTP* pHTTP,
			       const char* pData,
			       const unsigned int nLength,
			       const unsigned int nBlock,
			       aHTTPTemplateProc templateProc,
			       void* vpRef,
			       aStreamRef reply)
{
  aErr uiErr = aErrNone;
  aBool bDone = aTrue;
  aBool bWrite = aTrue;
  aStreamRef buffer = NULL;

  /* build the stream buffer for this block */
  if (uiErr == aErrNone)
    aStreamBuffer_Create(pHTTP->ioRef, aHTTPBUFFERINC, 
    			 &buffer, &uiErr);

  do {
    const char* p = pData;
    char num[10];
    unsigned int param;

    /* run through the buffer and call for substitutions */
    while ((uiErr == aErrNone) 
    	   && ((unsigned int)(p - pData) < nLength)) {
      if (*p == '^') {
        /* copy to the first non-digit */
        char* c = num;
        p++;
        while ((*p >= '0') && (*p <= '9') && (c - num < 9))
          *c++ = *p++;
        *c = 0;
        aIntFromString((int*)&param, num);
 
        if (templateProc)
          uiErr = templateProc(param, nBlock, buffer, vpRef);
 
        if (uiErr == aErrEOF) {
          uiErr = aErrNone;
          bDone = aTrue;
          bWrite = aFalse;
          break;

        /* block 0 is a special case for the global block and 
         * it never repeats */
        } else if (nBlock)
          bDone = aFalse;

      } else {
        /* handle escaped newlines */
        if ((p[0] == '\\') && (p[1] == 'n')) {
          aStream_Write(pHTTP->ioRef, buffer, "\n", 1, &uiErr);
          p += 2;
        } else {
          aStream_Write(pHTTP->ioRef, buffer, p, 1, &uiErr);
          p++;
        }
      }
    } /* while */
    
    /* write the buffer or clear it */
    if ((uiErr == aErrNone) && bWrite)
      uiErr = sMemFlush(pHTTP->ioRef, buffer, reply);      
    else
      aStream_Flush(pHTTP->ioRef, buffer, NULL, &uiErr);

  } while (!bDone);

  /* always clean up the buffer */
  if (buffer)
    aStream_Destroy(pHTTP->ioRef, buffer, (aErr*)NULL);

  return uiErr;

} /* sHTTP_WriteTemplateBuffer */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHTTP_HandleTemplate
 */

aErr sHTTP_HandleTemplate(aHTTP* pHTTP,
			  aStreamRef tpl,
			  const char** remainder,
			  const unsigned int nBlock,
			  aHTTPTemplateProc templateProc,
			  void* vpRef,
			  aStreamRef reply)
{
  aErr uiErr = aErrNone;
  char line[aHTTPMAXTEMPLATELINE];
  const char* p;
  const char* r;
  char key[aHTTPMAXTEMPLATELINE];
  aMemSize size;
  char* data;
  aStreamRef buffer = NULL;


  /* build the stream buffer for this level */
  if (uiErr == aErrNone)
    aStreamBuffer_Create(pHTTP->ioRef, aHTTPBUFFERINC, 
    			 &buffer, &uiErr);

  /* now, read bytes into the buffer until you get to a <BLOCK> 
   * tag or a parameter */
  if (remainder)
    p = *remainder;
  else
    p = "";

  while (uiErr == aErrNone) {

    /* get the next template line ignoring blank lines */
    while ((uiErr == aErrNone) && (*p == 0)) {

      /* reset the remainder */
      if (remainder)
        *remainder = 0;
      aStream_ReadLine(pHTTP->ioRef, tpl, line, 
    		       aHTTPMAXTEMPLATELINE, &uiErr);
      p = line;
    }

    /* step through the line and look for magic things */
    while (*p && (uiErr == aErrNone)) {

      /* a parameter specification */
      r = 0;
      switch (*p) {

      /* handle the special case for \n character specification */
      case '\\':
        if (p[1] == 'n') {
          aStream_Write(pHTTP->ioRef, buffer, "\n", 1, &uiErr);
          p += 2;
        }
        break;

      /* if we see a tag start, make sure it isn't for us */
      case '<':
        r = aString_CopyToChar(key, p, '>');
          
        /* if it is a BLOCK start, handle it */
        if (!aStringCompare(key, "<BLOCK")) {
          
          /* get the next block number */
          pHTTP->nTemplateBlock++;

	  /* get the buffer in the current block up to this
	   * nested block call */
	  aStreamBuffer_Get(pHTTP->ioRef, buffer, &size, 
	    		      &data, &uiErr);

          /* dump any existing buffer up to the block */
          if ((uiErr == aErrNone) && size)
            uiErr = sHTTP_WriteTemplateBuffer(pHTTP, 
            		    data, (unsigned int)size, 
            		    nBlock,
            		    templateProc, vpRef, reply);

          /* clear the buffer */
          if (uiErr == aErrNone)
            aStream_Flush(pHTTP->ioRef, buffer, 
             			NULL, &uiErr);

	  /* go recursively handle the contents of the nested 
	   * block */
          if (uiErr == aErrNone)
            uiErr = sHTTP_HandleTemplate(pHTTP, tpl, &r, 
            				 pHTTP->nTemplateBlock,
            				 templateProc,
            				 vpRef,
            				 reply);

	  /* make sure the current line has no remainder */
          if (r) {
            p = r;
            break;
          }

	/* a close block should finish out this current BLOCK 
	 * scope keeping track of any remaining line contents */
        } else if (!aStringCompare(key, "</BLOCK")) {
          if (remainder)
            *remainder = r;
          goto end;
        }
        
        /* fallthrough */

      /* the default case is to pass the character through to
       * the buffer */
      default:
        aStream_Write(pHTTP->ioRef, buffer, p, 1, &uiErr);
        p++;
        break;

      } /* switch */
    } /* while */
  } /* while */

  /* block ends show as EOF */
  if (uiErr == aErrEOF)
    uiErr = aErrNone;

end:
  /* see what we have left */
  if (uiErr == aErrNone)
    aStreamBuffer_Get(pHTTP->ioRef, buffer, &size, &data, &uiErr);

  /* write any remaining buffer */
  if ((uiErr == aErrNone) && size)
    sHTTP_WriteTemplateBuffer(pHTTP, data, (unsigned int)size, 
    			      nBlock, templateProc, vpRef, reply);

  /* always clean up the buffer storage */
  if (buffer)
    aStream_Destroy(pHTTP->ioRef, buffer, (aErr*)NULL);

  return uiErr;

} /* sHTTP_HandleTemplate */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHTTP_DisplayMsg
 */

aErr sHTTP_DisplayMsg(aHTTP* pHTTP,
		      aStreamRef reply,
		      const char* message)
{
  aErr uiErr = aErrNone;
  
  if (uiErr == aErrNone)
    aStream_Flush(pHTTP->ioRef, reply, NULL, &uiErr);
  
  if (uiErr == aErrNone)
    aStream_Write(pHTTP->ioRef, reply,
		  "<HTML><BR><BR><TABLE"
	   	  " ALIGN='CENTER' CELL"
		  "SPACING='0' BORDER='"
		  "1'><TR ALIGN='CENTER"
		  "'><TD BGCOLOR='#CCCC"
		  "99'><FONT FACE='Helv"
		  "etica,Arial' SIZE='5"
		  "' COLOR='WHITE'><B>",
                  159, &uiErr);
  if (uiErr == aErrNone)
    aStream_Write(pHTTP->ioRef, reply,
    		  message, aStringLen(message), &uiErr);
  if (uiErr == aErrNone)
    aStream_Write(pHTTP->ioRef, reply,
		  "</B></FONT></TD></TR"
		  "><TR ALIGN='CENTER'>"
		  "<TD><FONT FACE='Helv"
		  "etica,Arial' SIZE='3"
		  "'>Acroname aHTTP Ser"
		  "ver</FONT></TD></TR>"
		  "</TABLE></HTML>",
    		  135, &uiErr);

  return uiErr;

} /* sHTTP_DisplayMsg */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aHTTP_Create
 */

aLIBRETURN aHTTP_Create(
  aUILib uiRef,
  aSettingFileRef settings,
  aHTTPRequestProc requestProc,
  void* vpRef,
  aHTTPRef* pHTTPRef,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aHTTP* pHTTP = (aHTTP*)NULL;
#ifndef aPALM
  int i;
#endif /* aPALM */

  aVALIDUI(uiRef);

  /* allocate and initialize the web server structure */
  if (uiErr == aErrNone) {
    pHTTP = (aHTTP*)aMemAlloc(sizeof(aHTTP));
    if (pHTTP) {
      aBZero(pHTTP, sizeof(aHTTP));
      pHTTP->requestProc = requestProc;
      pHTTP->vpRef = vpRef;
      pHTTP->check = aHTTPCHECK;
    } else
      uiErr = aErrMemory;
  }

  if (uiErr == aErrNone)
    aIO_GetLibRef(&pHTTP->ioRef, &uiErr);

  /* create a setting file if none was passed in */
  if (uiErr == aErrNone) {
    if (settings)
      pHTTP->settings = settings;
    else {
      aSettingFile_Create(pHTTP->ioRef, aHTTPSETTINGMAX, 
      			  aHTTPSETTINGFILE,
                          &pHTTP->settings, &uiErr);
      pHTTP->bCreatedSettings = aTrue;
    }
  }

  aAssert(uiErr == aErrNone);

  /* get the port number from the setting file */
  if (uiErr == aErrNone) {
    int num;
    aAssert(pHTTP->settings);
    aSettingFile_GetInt(pHTTP->ioRef, pHTTP->settings,
			aHTTPPORTKEY, &num, 
			aHTTPPORTDEFAULT, &uiErr);
    if (uiErr == aErrNone)
      pHTTP->nPort = (unsigned short)num;
  }

  /* build the memory pool for the HTTP parameter values */
  if (uiErr == aErrNone)
    aMemPool_Create(pHTTP->ioRef, 
     		    aMAXIDENTIFIERLEN,
     		    8,
     		    &pHTTP->paramPool,
     		    &uiErr);

  /* create a server log file */
  if (uiErr == aErrNone) {
    acpString filename("aHTTP-");
    filename += pHTTP->nPort;
    filename += ".log";
    aStream_CreateFileOutput(pHTTP->ioRef, 
    			     filename, 
    			     aFileAreaBinary,
    			     &pHTTP->log,
    			     &uiErr);
  }

  /* insert a simple message */
  if (uiErr == aErrNone)
    uiErr = sHTTP_LogLine(pHTTP, "aHTTP Starting");

  /* show where we are getting the settings */
  if (uiErr == aErrNone) {
    if (pHTTP->bCreatedSettings) {
      uiErr = sHTTP_LogLine(pHTTP, "using " aHTTPSETTINGFILE " settings");
    } else {
      uiErr = sHTTP_LogLine(pHTTP, "using passed in settings");
    }
  }

  /* get the address to use */
#ifndef aPALM
  if (uiErr == aErrNone) {
    unsigned long machineAddr;
    acpString url;

    /* find the address of this machine */
    aIO_GetInetAddr(pHTTP->ioRef, &machineAddr, &uiErr);

    aAssert(uiErr == aErrNone);

    /* override the address if a setting is present */
    if (uiErr == aErrNone)
      aSettingFile_GetInetAddr(pHTTP->ioRef, pHTTP->settings,
			       aHTTPADDRKEY, &pHTTP->nInetAddr,
			       machineAddr, &uiErr);

    if (uiErr == aErrNone) {
      char num[20];
      if (!aUtil_FormatInetAddr(num, pHTTP->nInetAddr, &uiErr)) {
	url += num;
	url += ':';
	url += pHTTP->nPort;
	uiErr = sHTTP_LogLine(pHTTP, url);
      }
    }
  }

  /* open up the server sockets */
  for (i = 0; (i < aHTTPNUMPORTS) && (uiErr == aErrNone); i++)
    uiErr = sHTTP_RebuildStream(pHTTP, i);

  /* build the symbol table for cached assets */
  if (uiErr == aErrNone)
    aSymbolTable_Create(pHTTP->ioRef, 
      			&pHTTP->assetCache,
      			&uiErr);

  /* build the stream buffer for the HTTP reply headers */
  if (uiErr == aErrNone)
    aStreamBuffer_Create(pHTTP->ioRef, 
    		         aHTTPBUFFERINC,
    		         &pHTTP->headerBuffer,
    		         &uiErr);

  /* build the stream buffer for the HTTP reply data */
  if (uiErr == aErrNone)
    aStreamBuffer_Create(pHTTP->ioRef, 
    		         aHTTPBUFFERINC,
    		         &pHTTP->replyBuffer,
    		         &uiErr);
#endif /* aPALM */

  if (uiErr == aErrNone) {
    *pHTTPRef = pHTTP;

  /* free the resources if we had problems */
  } else {
    if (pHTTP) {
      if (pHTTP->ioRef)
        aIO_ReleaseLibRef(pHTTP->ioRef, (aErr*)NULL);
      aMemFree(pHTTP);
    }
  }

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aHTTP_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aHTTP_TimeSlice
 */

aLIBRETURN aHTTP_TimeSlice(
  aUILib uiRef,
  aHTTPRef http,
  aBool* bChanged,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aHTTP* pHTTP = (aHTTP*)http;
  char line[aHTTPMAXREQESTLINE];

  aVALIDUI(uiRef);
  if (uiErr == aErrNone)
    aVALIDHTTP(http);

  if ((uiErr == aErrNone) && bChanged)
    *bChanged = aFalse;

  /* get a line */
  if ((uiErr == aErrNone) && (pHTTP->ports[pHTTP->nCur])) {
    aErr readErr;
    aStream_ReadLine(pHTTP->ioRef, pHTTP->ports[pHTTP->nCur], 
    		     line, aHTTPMAXREQESTLINE, &readErr);
    switch (readErr) {

    /* we got a valid reqest line so handle it */
    case aErrNone:
      uiErr = sHTTP_HandleRequest(pHTTP, line);
      if (bChanged)
        *bChanged = aTrue;
      break;
    
    case aErrNotReady:
    case aErrEOF:
      break;

    default:
      aAssert(0);
      break;

    } /* switch */
  }

  /* rotate through the HTTP ports */  
  if (uiErr == aErrNone) {
    pHTTP->nCur++;
    if (pHTTP->nCur == aHTTPNUMPORTS)
      pHTTP->nCur = 0;
  }

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aHTTP_TimeSlice */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aHTTP_Template
 * 
 * This function opens the requested template and injects it into
 * the reply stream provided calling the template callback for 
 * each parameter.  The parameters are embedded in the template
 * as a '^' character followed by the numeric parameter index. 
 * These indices are zero-based.  For example, "^3" indicates
 * the 4th parameter and the calleback will called with index 3
 * to supply the substitution text for this parameter. Template
 * blocks can be specified which will be repeated until the 
 * callback returns aErrEOF.
 */

aLIBRETURN aHTTP_Template(aUILib uiRef,
			  aHTTPRef http,
			  const char* pName,
			  aHTTPTemplateProc templateProc,
			  void* vpRef,
			  aStreamRef reply,
			  aErr* pErr)
{
  aErr uiErr = aErrNone;
  aHTTP* pHTTP = (aHTTP*)http;
  aStreamRef tpl = NULL;

  aVALIDUI(uiRef);
  if (uiErr == aErrNone)
    aVALIDHTTP(http);
  if ((uiErr == aErrNone) 
      && (!pName))
    uiErr = aErrParam;

  /* try to open the template file */
  if (uiErr == aErrNone)
    aStream_CreateFileInput(pHTTP->ioRef, pName, aFileAreaAsset, 
    	                    &tpl, &uiErr);


  if (uiErr == aErrNone) {
    pHTTP->nTemplateBlock = 0;
    uiErr = sHTTP_HandleTemplate(pHTTP,
    				 tpl,
    				 (const char**)NULL,
    				 (const unsigned int)0,
    				 templateProc,
    				 vpRef,
    				 reply);
  }

  /* always close the open file */
  if (tpl)
    aStream_Destroy(pHTTP->ioRef, tpl, (aErr*)NULL);

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aHTTP_Template */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aHTTP_Destroy
 */

aLIBRETURN aHTTP_Destroy(
  aUILib uiRef,
  aHTTPRef http,
  aErr* pErr
)
{
  aErr uiErr = aErrNone;
  aHTTP* pHTTP = (aHTTP*)http;
  int i;

  aVALIDUI(uiRef);
  if (uiErr == aErrNone)
    aVALIDHTTP(http);

  /* insert a simple message */
  if (uiErr == aErrNone)
    uiErr = sHTTP_LogLine(pHTTP, "aHTTP Stopping");

  if ((uiErr == aErrNone) && pHTTP->log) {
    aStream_Destroy(pHTTP->ioRef, pHTTP->log, (aErr*)NULL);
    pHTTP->log = NULL;
  }

  if ((uiErr == aErrNone) && pHTTP->paramPool) {
    aMemPool_Destroy(pHTTP->ioRef, pHTTP->paramPool, (aErr*)NULL);
    pHTTP->paramPool = NULL;
  }

  for (i = 0; (uiErr == aErrNone) && (i < aHTTPNUMPORTS); i++) {
    if (pHTTP->ports[i]) {
      aStream_Destroy(pHTTP->ioRef, pHTTP->ports[i], (aErr*)NULL);
      pHTTP->ports[i] = NULL;
    }
  }

  /* clean up the asset cache */
  if ((uiErr == aErrNone) && pHTTP->assetCache) {
    aSymbolTable_Destroy(pHTTP->ioRef, pHTTP->assetCache, (aErr*)NULL);
    pHTTP->assetCache = NULL;
  }

  /* clean up the header buffer */
  if ((uiErr == aErrNone) && pHTTP->headerBuffer) {
    aStream_Destroy(pHTTP->ioRef, pHTTP->headerBuffer, (aErr*)NULL);
    pHTTP->headerBuffer = NULL;
  }

  /* clean up the reply buffer */
  if ((uiErr == aErrNone) && pHTTP->replyBuffer) {
    aStream_Destroy(pHTTP->ioRef, pHTTP->replyBuffer, (aErr*)NULL);
    pHTTP->replyBuffer = NULL;
  }

  /* if we created a setting file, clean it up */
  if ((uiErr == aErrNone) && pHTTP->settings) {
    if  (pHTTP->bCreatedSettings)
      aSettingFile_Destroy(pHTTP->ioRef, pHTTP->settings, (aErr*)NULL);
    pHTTP->settings = NULL;
  }

  /* clean up the aIO library reference */
  if ((uiErr == aErrNone) && pHTTP->ioRef) {
    aIO_ReleaseLibRef(pHTTP->ioRef, (aErr*)NULL);
    pHTTP->ioRef = NULL;
  }

  /* invalidate and clean up the actual HTTP record */
  if (uiErr == aErrNone) {
    pHTTP->check = 0; 
    aMemFree(pHTTP);
  }

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aHTTP_Destroy */
