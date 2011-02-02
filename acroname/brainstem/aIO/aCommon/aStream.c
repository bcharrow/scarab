/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStream.c                                                 */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Cross-Platform implementation of stream I/O        */
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

#include "aIO.h"
#include "aStream.h"
#include "aIOInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * defaults for creating a stream from settings
 */

#define LINKTYPEKEY             "linktype"
#define DEFAULTLINKTYPE         "serial"
#define BAUDRATEKEY		"baudrate"
#define PORTNAMEKEY		"portname"
#define DEFAULTBAUDRATE		9600
#define USBSERIALKEY            "usb_id"
#define MODULEKEY		"module"
#define IPADDRKEY		"ip_address"
#define DEFAULTIPADDR		(unsigned long)0x7F000001
#define IPPORTKEY		"ip_port"
#define DEFAULTIPPORT		8000
#ifdef aWIN
#define DEFAULTPORTNAME 	"COM1"
#endif /* aWIN */
#ifdef aUNIX
#ifdef aMACX
#define DEFAULTPORTNAME 	"tty.usbserial"
#else /* aMACX */
#define DEFAULTPORTNAME 	"ttyS0"
#endif /* aMACX */
#endif /* aUNIX */ 


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local routines for look ahead buffer used by stream read routines
 */

static aBool sStreamGetBufferedChar(aStream* pStream, 
				    char* pCharacter);
static aErr sStreamBufferChar(aStream* pStream, const char c);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * defines for file streams
 */
typedef struct aFileStreamData {
  aIOLib    ioRef;
  aFileRef  fileRef;
} aFileStreamData;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local static file stream callbacks
 */

static aErr sFileStreamGet(char* pData, void* ref);
static aErr sFileStreamPut(char* pData, void* ref);
static aErr sFileStreamWrite(const char* pData, 
		             const unsigned long nSize,
		             void* ref);
static aErr sFileStreamDelete(void* ref);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStreamGetBufferedChar
 */

aBool sStreamGetBufferedChar(aStream* pStream, 
			     char* pCharacter)
{
  if (pStream->bufCount == 0)
    return aFalse;

  /* retrieve the character from the buffer */
  *pCharacter = pStream->pBuffer[pStream->bufFirst];
  pStream->bufFirst = (pStream->bufFirst + 1) % pStream->bufSize;
  pStream->bufCount--;

  return aTrue;

} /* sStreamGetBufferedChar */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStreamBufferChar
 */

aErr sStreamBufferChar(aStream* pStream, const char c)
{
  aErr err = aErrNone;
  
  aAssert(pStream);

  /* if no buffer is present, build one */
  if (pStream->pBuffer == NULL) {
    pStream->pBuffer = (char*)aMemAlloc(aSTREAMBUFCHUNK);
    if (pStream->pBuffer == NULL)
      err = aErrMemory;
    else {
      pStream->bufSize = aSTREAMBUFCHUNK;
      pStream->bufCount = 0;
      pStream->bufLast = 0;
      pStream->bufFirst = 0;
    }
  }

  aAssert(pStream->pBuffer);

  /* now, see if the buffer is full */
  if ((err == aErrNone)
      && (pStream->bufCount >= pStream->bufSize - 1)) {
    /* grow the buffer */
    char* pNew = (char*)aMemAlloc((aMemSize)(pStream->bufSize 
    					     + aSTREAMBUFCHUNK));
    if (pNew == NULL)
      err = aErrMemory;
    else {
      unsigned int c1Size, c2Size;

      /* copy from the first to the end of the existing */
      c1Size = ((pStream->bufLast > pStream->bufFirst) ?
                	pStream->bufLast : 
                	pStream->bufSize) - pStream->bufFirst;
      aMemCopy(pNew, &pStream->pBuffer[pStream->bufFirst], c1Size);
      
      /* now, handle any wrap */
      if (pStream->bufLast < pStream->bufFirst) {
        c2Size = pStream->bufLast;
        if (c2Size)
          aMemCopy(&pNew[c1Size], pStream->pBuffer, c2Size);
      } else
        c2Size = 0;
      
      /* now, switch out the smaller buffer for the larger copy */
      pStream->bufFirst = 0;
      pStream->bufLast = c1Size + c2Size;
      aAssert(pStream->bufCount == pStream->bufLast);
      pStream->bufSize = pStream->bufSize + aSTREAMBUFCHUNK;
      aMemFree(pStream->pBuffer);
      pStream->pBuffer = pNew;
    }
  }

  /* add the character to the buffer */
  if (err == aErrNone) {
    aAssert(pStream->pBuffer);
    pStream->pBuffer[pStream->bufLast] = c;
    pStream->bufLast = (pStream->bufLast + 1) % pStream->bufSize;
    pStream->bufCount++;
  }

  return err;

} /* sStreamBufferChar */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aFileStreamGet
 */

aErr sFileStreamGet(char* pData,
		    void* ref)
{
  aErr err = aErrNone;
  aFileStreamData* data = (aFileStreamData*)ref;

  if (data == NULL)
    err = aErrIO;
  else
    aFile_Read(data->ioRef, data->fileRef, pData, 1, NULL, &err);

  return(err);

} /* end of sFileStreamGet */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sFileStreamPut
 */

aErr sFileStreamPut(char* pData,
		    void* ref)
{
  aErr err = aErrNone;
  aFileStreamData* data = (aFileStreamData*)ref;

  if (data == NULL)
    err = aErrIO;
  else {
    aFile_Write(data->ioRef, data->fileRef, pData, 1, NULL, &err);
  }

  return(err);

} /* end of sFileStreamPut */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sFileStreamWrite
 */

aErr sFileStreamWrite(const char* pData, 
		      const unsigned long nSize,
		      void* ref)
{
  aErr err = aErrNone;
  aFileStreamData* data = (aFileStreamData*)ref;

  /* check for valid stream data */
  aFile_Write(data->ioRef, data->fileRef, pData, nSize, NULL, &err);

  return err;

} /* sFileStreamWrite */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aFileStreamDelete
 */

aErr sFileStreamDelete(void* ref)
{
  aErr err = aErrNone;
  aFileStreamData* data = (aFileStreamData*)ref;

  if (data == NULL)
    err = aErrIO;
  else {
    aFile_Close(data->ioRef, data->fileRef, &err);
    aMemFree((aMemPtr)data);
  }

  return(err);

} /* end of sFileStreamDelete */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_CreateFromSettings
 */

aLIBRETURN
aStream_CreateFromSettings(aIOLib ioRef, 
			   aSettingFileRef settings,
			   aStreamRef* pStreamRef,
			   aErr* pErr)
{
  aErr err = aErrNone;
  aStreamRef linkStream = NULL;
  char* linktype;
  
  if (err == aErrNone)
    aSettingFile_GetString(ioRef, 
    			   settings, 
    			   LINKTYPEKEY, 
    			   &linktype,
     			   DEFAULTLINKTYPE, 
     			   &err);
  
  if (err == aErrNone) {

    if (!aStringCompare(linktype, "serial")) {
      char* portname;
      int baudRate;
      
      if (err == aErrNone)
	aSettingFile_GetInt(ioRef, 
			    settings, 
			    BAUDRATEKEY, 
			    &baudRate,
			    DEFAULTBAUDRATE, 
			    &err);
      
      if (err == aErrNone)
	aSettingFile_GetString(ioRef, 
			       settings, 
			       PORTNAMEKEY, 
			       &portname,
			       DEFAULTPORTNAME, 
			       &err);
      if (err == aErrNone) {
	aStream_CreateSerial(ioRef,
			     portname,
			     (unsigned int)baudRate,
			     &linkStream,
			     &err);
      }

    } else if (!aStringCompare(linktype, "usb")) {
      int serialnum;
      
      if (err == aErrNone)
	aSettingFile_GetInt(ioRef, 
			    settings, 
			    USBSERIALKEY, 
			    &serialnum,
			    0, 
			    &err);
      if (err == aErrNone) {
	if (serialnum) {
	  aStream_CreateUSB(ioRef, serialnum, &linkStream, &err);
	  
	  /* error description here? */
	}
#if 0
	else {
	  aDialog_Message(uiRef,
			  "USB ID not set in config file",
			  NULL, NULL, NULL);
	}
#endif
      }
      
    } else if (!aStringCompare(linktype, "ip")) {
      unsigned long address;
      int port;
      if (err == aErrNone) {
        aSettingFile_GetInetAddr(ioRef, settings, 
    			         IPADDRKEY, &address,
     			         DEFAULTIPADDR, &err);
      }
      if (err == aErrNone) {
        aSettingFile_GetInt(ioRef, settings, 
    			    IPPORTKEY, &port,
     			    DEFAULTIPPORT, &err);
      }
      aStream_CreateSocket(ioRef,
    			   address,
    			   (unsigned short)port,
    			   aFalse,
    			   &linkStream,
    			   &err);
    }
  }

  *pStreamRef = (err == aErrNone) ? linkStream : NULL;

  if (pErr)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);  
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_Create
 */

aLIBRETURN 
aStream_Create(aIOLib ioRef,
	       aStreamGetProc getProc,
	       aStreamPutProc putProc,
	       aStreamDeleteProc deleteProc,
	       const void* procRef,
	       aStreamRef* pStreamRef,
	       aErr* pErr)
{
  aErr err = aErrNone;
  aStream* pStream;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params and initialize
   */
  if (pStreamRef == NULL)
    err = aErrParam;
  else
    *pStreamRef = NULL;

  if (err == aErrNone) {
    pStream = (aStream*)aMemAlloc(sizeof(aStream));
    if (pStream == NULL) {
      err = aErrMemory;
    } else {
     aBZero(pStream, sizeof(aStream));
     pStream->getProc = getProc;
     pStream->putProc = putProc;
     pStream->deleteProc = deleteProc;
     pStream->procRef = (void*)procRef;
     pStream->check = aSTREAMCHECK;
     pStream->libRef = ioRef;
     *pStreamRef = pStream;
    }
  }

  if (pErr != NULL)
    *pErr = err;

  return((aLIBRETURN)(err != aErrNone));

} /* end of aStream_Create routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_CreateFileInput
 */

aLIBRETURN 
aStream_CreateFileInput(aIOLib ioRef, 
			const char *pFilename,
			const aFileArea eArea,
			aStreamRef* pStreamRef,
			aErr* pErr)
{
  aFileRef inputFile = NULL;
  aStreamRef streamRef = NULL;
  aFileStreamData* fileData;
  aErr err = aErrNone;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params
   */
  if ((pFilename == NULL) || (pStreamRef == NULL))
    err = aErrParam;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * try to open the file
   */
  if (err == aErrNone)
    aFile_Open(ioRef, pFilename, aFileModeReadOnly, eArea,
    	       &inputFile, &err);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * create the private stream data
   */
  if (err == aErrNone) {
    fileData = (aFileStreamData*)aMemAlloc(sizeof(aFileStreamData));
    if (fileData == NULL)
      err = aErrMemory;
    else {
      fileData->ioRef = ioRef;
      fileData->fileRef = inputFile;
    }
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * create the stream
   */
  if (err == aErrNone)
    aStream_Create(ioRef, 
    		   sFileStreamGet,
    		   NULL,
    		   sFileStreamDelete,
    		   fileData,
    		   &streamRef, 
    		   &err);

  if (err == aErrNone)
    *pStreamRef = streamRef;
  else {
    if (inputFile != NULL) {
      aFile_Close(ioRef, inputFile, NULL);
    }
  }

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);
  
} /* aStream_CreateFileInput */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_CreateFileOutput
 */

aLIBRETURN 
aStream_CreateFileOutput(aIOLib ioRef,
			 const char *pFilename,
			 const aFileArea eArea,
			 aStreamRef* pStreamRef,
			 aErr* pErr)
{
  aErr err = aErrNone;
  aStreamRef streamRef;
  aFileRef outputFile;
  aFileStreamData* fileData;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params
   */
  if ((pFilename == NULL) || (pStreamRef == NULL))
    err = aErrParam;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * try to open the file
   */
  if (err == aErrNone) {
    aFile_Open(ioRef, pFilename, aFileModeWriteOnly, eArea,
    	       &outputFile, &err);
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * create the private file stream data
   */
  if (err == aErrNone) {
    fileData = (aFileStreamData*)aMemAlloc(sizeof(aFileStreamData));
    if (fileData == NULL)
      err = aErrMemory;
    else {
      fileData->ioRef = ioRef;
      fileData->fileRef = outputFile;
    }
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * build the actual stream
   */
  if (err == aErrNone) {
    aStream_Create(ioRef, 
    		   NULL,
    		   sFileStreamPut,
    		   sFileStreamDelete,
    		   fileData,
    		   &streamRef, 
    		   &err);
    if (err != aErrNone)
      aFile_Close(ioRef, outputFile, NULL);
  }

  if (err == aErrNone) {
    aStream* pStream = (aStream*)streamRef;
    pStream->writeProc = sFileStreamWrite;
    *pStreamRef = streamRef;
  }
  
  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* end of aStream_CreateFileOutput */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_Read
 */

aLIBRETURN 
aStream_Read(aIOLib ioRef, 
	     aStreamRef streamRef,
	     char* pBuffer,
	     const unsigned long ulLength,
	     aErr* pErr)
{
  unsigned long i;
  aErr err = aErrNone;
  char* p = pBuffer;
  aStream* pStream = (aStream*)streamRef;
  
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params
   */
  aVALIDSTREAM(pStream);
  if (pBuffer == NULL)
    err = aErrParam;
  if ((err == aErrNone) && (pStream->getProc == NULL))
    err = aErrMode;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * do the read
   */
  for (i = 0; (err == aErrNone) && (i < ulLength); i++, p++)
    err = pStream->getProc(p, pStream->procRef);

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* end of aStream_Read */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_Write
 */

aLIBRETURN 
aStream_Write(aIOLib ioRef, 
	      aStreamRef streamRef,
	      const char* pBuffer,
	      const unsigned long ulLength,
	      aErr* pErr)
{
  aErr err = aErrNone;
  unsigned long i;
  char *p;
  aStream* pStream = (aStream*)streamRef;


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params
   */
  aVALIDSTREAM(pStream);
  aAssert(err == aErrNone);
  if (pBuffer == NULL)
    return(aErrParam);
  if (pStream->putProc == NULL)
    return(aErrMode);

  if ((pStream->writeProc != NULL) && (ulLength > 1))
    err = pStream->writeProc(pBuffer, ulLength, pStream->procRef);
  else {
    p = (char *)pBuffer;
    for (i = 0; (err == aErrNone) && (i < ulLength); i++, p++) 
      err = pStream->putProc(p, pStream->procRef);
  }

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStream_Write */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_ReadLine
 *
 *  For the record:
 *   Mac text lines end with 0x0D ("\r")
 *   Unix text lines end with 0x0A ("\n")
 *   DOS text lines end with 0x0D 0x0A ("\r\n")
 */

aLIBRETURN 
aStream_ReadLine(aIOLib ioRef, 
		 aStreamRef streamRef,
		 char* pBuffer,
		 const unsigned long ulLength,
		 aErr* pErr)
{
  aErr err = aErrNone;
  aErr copyErr;
  unsigned long ulCount = 0;
  char in, ahead;
  aBool bDone = aFalse;
  aStream* pStream = (aStream*)streamRef;
  unsigned int i;


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params
   */
  aVALIDSTREAM(pStream);
  if ((pBuffer == NULL) || (ioRef == NULL))
    err = aErrParam;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * need at least one byte for null termination
   */
  if ((err == aErrNone) && (ulLength < 1))
    err = aErrParam;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * get characters until we reach EOF, not ready, or end of line
   */
  while ((err == aErrNone)
         && (bDone == aFalse)
         && (ulCount < ulLength)) {

    /* get the next character */
    if (!sStreamGetBufferedChar(pStream, &in))
      aStream_Read(ioRef, streamRef, &in, 1, &err);

    switch (err) {

    /* EOF means we are done */
    case aErrEOF:
      /* the last line could end in EOF */
      if (ulCount != 0) {
        err = aErrNone;
        bDone = aTrue;
      }
      break;

    /* not ready means the data isn't here yet */
    case aErrNotReady:
      /* buffer what we have so far */
      copyErr = aErrNone;
      for (i = 0; (copyErr == aErrNone) && (i < ulCount); i++)
        copyErr = sStreamBufferChar(pStream, pBuffer[i]);
      break;
    
    case aErrNone:
      switch (in) {

      case '\n':
        /* eat the newline and bail */
        bDone = aTrue; 
        break;

      case '\r':
        /* look ahead at the next character */
        if (!sStreamGetBufferedChar(pStream, &ahead))
          aStream_Read(ioRef, streamRef, &ahead, 1, &err);

        if (err == aErrNone) {

          /* gobble the \r\n and we are done */
          if (ahead == '\n') {
            bDone = aTrue;
            break;
          }
          /* next char seems valid start of line, we are done */
          bDone = aTrue;
          err = sStreamBufferChar(pStream, ahead);

        /* special case for files that end in an \r */
        } else if (err == aErrEOF) {
          err = aErrNone;
          bDone = aTrue;
        }
        break;

      default:
        pBuffer[ulCount++] = in;
        break;

      } /* end switch */
      break;

    default:
      break;
 
    } /* switch */
  }

  if (err == aErrNone)
    pBuffer[ulCount] = '\0';   /* null terminate */

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* end of aStream_ReadLine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_WriteLine
 */

aLIBRETURN 
aStream_WriteLine(aIOLib ioRef, 
		  aStreamRef streamRef,
		  const char* pBuffer,
		  aErr* pErr)
{
  aErr err = aErrNone;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params
   */
  if ((pBuffer == NULL) || (ioRef == NULL))
    err = aErrParam;
  aVALIDSTREAM(streamRef);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * write the line
   */
  if (err == aErrNone)
    aStream_Write(ioRef, streamRef, pBuffer, 
  		  aStringLen(pBuffer), &err);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * write line terminator (aszEOL is OS-specific, see OSDefs.h)
   */
  if (err == aErrNone)
    aStream_Write(ioRef, streamRef, aszEOL, 
  		  aStringLen(aszEOL), &err);

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStream_WriteLine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_Flush
 */

aLIBRETURN aStream_Flush(aIOLib ioRef,
			 aStreamRef inStreamRef,
			 aStreamRef outStreamRef,
			 aErr* pErr)
{
  aErr err = aErrNone;
  aErr getErr;
  aStream* pStream = (aStream*)inStreamRef;
  aStream* pFlush = (aStream*)outStreamRef;

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

  /* make sure we are able to read from the buffer and write to
   * the flush */
  if (err == aErrNone) {
    if (pStream->getProc) {
      if (pFlush && !pFlush->putProc)
        err = aErrMode;
    } else
      err = aErrMode;
  }

  /* now read from the buffer and write to the flush (when 
   * present) */
  getErr = aErrNone;
  while ((getErr == aErrNone) && (err == aErrNone)) {
    char c;
    getErr = pStream->getProc(&c, pStream->procRef);
    if (getErr == aErrNone) {
      if (pFlush) {
        err = pFlush->putProc(&c, pFlush->procRef);
      }
    } else if (getErr != aErrEOF)
      err = getErr;
  }

  if (pErr)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStream_Flush */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_Destroy
 */

aLIBRETURN 
aStream_Destroy(aIOLib ioRef,
		aStreamRef streamRef,
		aErr* pErr)
{
  aErr err = aErrNone;
  aStream* pStream = (aStream*)streamRef;
  aBool bValid = aFalse;

  /* check params */
  if (!ioRef)
    err = aErrParam;
  
  if (err == aErrNone) {
    aVALIDSTREAM(pStream);
  }
  
  /* stream memory is valid and can be freed */
  if (err == aErrNone) {
    bValid = aTrue;
  }
  
  /* delete proc may have error to send back to caller */
  /* it should only matter for stream specifics */
  /* general stream memory can still be freed */
  if ((err == aErrNone) && (pStream->deleteProc != NULL))
    err = pStream->deleteProc(pStream->procRef);

  /* clean up any buffer that may have been allocated */
  if (bValid
      && (pStream->pBuffer != NULL)) {
    aMemFree(pStream->pBuffer);
    pStream->pBuffer = NULL;
  }

  /* clean up the actual stream object */
  if (bValid) {
    pStream->check = 0; /* invalidate the check */
    aMemFree((aMemPtr)pStream);
  }

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* end of aStream_Destroy */
