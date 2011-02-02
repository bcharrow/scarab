/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aFile.c                                              */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: implementation of file I/O routines for win32.     */
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

#ifdef aUNIX

#include <unistd.h>
#include <errno.h>
#include <sys/param.h>

#include "aOSDefs.h"
#include "aIO.h"
#include "unix_aIO.h"
#include "unix_aIOUtils.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Local windows specific file structure.  This is hidden by the 
 * opaque reference type aSLFile.
 */

typedef struct aUnixFile {
  FILE*                         fp;
  aFileMode                     eMode;			
  int                           check;
} aUnixFile;

#define nFILECHECK		0xFEED

#define FILECHECK(fp) if ((fp == NULL) || 			   \
			  (((aUnixFile*)fp)->check != nFILECHECK)) \
			{ 					   \
			  err = aErrParam;			   \
			}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aFile_Open
 */

aLIBRETURN 
aFile_Open(aIOLib libRef,
	   const char *pFilename,
	   const aFileMode eMode,
	   const aFileArea eArea,
	   aFileRef* pFileRef,
	   aErr* pErr)
{
  aUnixFile* pFile = NULL;
  aErr err = aErrNone;
  char fullpath[aMAXUNIXFILECHARS];


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check the parameters
   */
  aVALIDIO(libRef);
  if ((err == aErrNone)
      && ((pFilename == NULL) 
          || (libRef == NULL) 
          || (pFileRef == NULL)))
    err = aErrParam;
  if ((err == aErrNone)
      && (eArea != aFileAreaNative)
      && (aStringLen(pFilename) > aFILE_NAMEMAXCHARS))
    err = aErrFileNameLength;

  if (err == aErrNone) {

    /* initialize the return value */
    *pFileRef = NULL;

    pFile = (aUnixFile*)aMemAlloc(sizeof(aUnixFile));
    if (pFile == NULL) {
      err = aErrMemory;
    } else {
      aBZero(pFile, sizeof(aUnixFile));
      pFile->check = nFILECHECK;

      /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       * Resolve the full path to the file
       */
      unix_aFileFullPath(fullpath, pFilename, eArea);

      switch (eMode) {
 
      case aFileModeReadOnly:
        pFile->fp = fopen(fullpath, "rb");
        if (pFile->fp == NULL) {
	  pFile->fp = 0;
	  err = aErrNotFound;
        }
        break;

      case aFileModeWriteOnly:
        pFile->fp = fopen(fullpath, "wb");
        if (pFile->fp == NULL) {
	  int e = errno;
	  pFile->fp = 0;
	  switch (e) {
	  case EBUSY:
	    err = aErrBusy;
	  default:
	    err = aErrIO;
	    break;
	  } /* switch */
        }
        break;
	
      default:
        aAssert(0); /* shouldn't have an undefined mode */
	break;

      } /* end of eMode switch */ 
    }

  } /* err = aErrNone */
  
  if (err == aErrNone) {
    pFile->eMode = eMode; 
    *pFileRef = pFile;
  } else {
    if (pFile != NULL)
      aMemFree(pFile);
  }

  if (pErr != NULL)
    *pErr = err;

  return err;

} /* aFile_Open */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aFile_Close
 */

aLIBRETURN 
aFile_Close(aIOLib libRef,
	    aFileRef fileRef,
	    aErr* pErr)
{
  aUnixFile* pFile = NULL;
  aErr err = aErrNone;

  aVALIDIO(libRef);
  if ((err == aErrNone) && (fileRef == NULL))
    err = aErrParam;

  if (err == aErrNone) {  
    pFile = (aUnixFile*)fileRef;
    FILECHECK(pFile);
  }

  if (err == aErrNone) {
    if (pFile->fp != NULL) {
      fclose(pFile->fp);
      pFile->fp = NULL;
    }
    /* make the file stale */
    pFile->check = 0;
    aMemFree(pFile);
  }

  if (pErr != NULL)
    *pErr = err;
  
  return err;

} /* aFile_Close */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aFile_Read
 */

aLIBRETURN 
aFile_Read(aIOLib libRef,
	   aFileRef fileRef,
	   char* pBuffer,
	   const unsigned long ulLength,
	   unsigned long* pulActuallyRead,
	   aErr* pErr)
{
  aUnixFile* pFile = NULL;
  aErr err = aErrNone;
  size_t ItemsRead = 0;

  aVALIDIO(libRef);
  if (err == aErrNone) {
    pFile = (aUnixFile*)fileRef;
    FILECHECK(pFile);
  }

  if (err == aErrNone) {
    if (pBuffer == NULL)
      err = aErrParam;
    if ((err == aErrNone)
        && (pFile->eMode != aFileModeReadOnly))
      err = aErrMode;
  }
      
  if (err == aErrNone) {
        
    ItemsRead = fread(pBuffer, 1, ulLength, pFile->fp);
  
    /* tell them how much we read */
    if (pulActuallyRead != NULL)
      *pulActuallyRead = (unsigned long) ItemsRead;
    
    /* if we read nothing and they wanted something, EOF them */
    if ((ItemsRead == 0) && (ulLength > 0))
      err = aErrEOF;
  
  }

  if (pErr != NULL)
    *pErr = err;
  
  return err;

} /* aFile_Read */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aFile_Write
 */

aLIBRETURN 
aFile_Write(aIOLib libRef,
	    aFileRef fileRef,
	    const char* pBuffer,
	    const unsigned long ulLength,
	    unsigned long* pulActuallyWritten,
	    aErr* pErr)
{
  aUnixFile* pFile = NULL;
  aErr err = aErrNone;
  size_t ItemsWritten = 0;

  aVALIDIO(libRef);
  if (err == aErrNone) {
    pFile = (aUnixFile*)fileRef;
    FILECHECK(pFile);
  }

  if (err == aErrNone) {
    if (pBuffer == NULL)
      err = aErrParam;
    if ((err == aErrNone) && (pFile->eMode != aFileModeWriteOnly))
      err = aErrMode;
  }

  if (err == aErrNone) {

    ItemsWritten = fwrite(pBuffer, 1, ulLength, pFile->fp);

    /* tell them how much we wrote */
    if (pulActuallyWritten != NULL)
      *pulActuallyWritten = (unsigned long) ItemsWritten;
    
    /* if we wrote nothing and wanted to, error them */
    if ((ItemsWritten == 0) && (ulLength > 0))
      err = aErrWrite;

  }

  if (pErr != NULL)
    *pErr = err;
  
  return err;

} /* aFile_Write */  				  


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aFile_Seek
 */

aLIBRETURN 
aFile_Seek(aIOLib libRef,
	   aFileRef fileRef,
	   const long lOffset,
	   aBool bSeekFromStart,
	   aErr* pErr)
{
  aUnixFile* pFile = NULL;
  aErr err = aErrNone;
  aErr sizeErr = aErrNone;
  long lFilePtr = 0;
  long ulOrigFilePtr = 0;
  long ulFileSize = 0;
  long lNewPos;


  /* check params */
  aVALIDIO(libRef);
  if (err == aErrNone) {
    pFile = (aUnixFile*)fileRef;
    FILECHECK(pFile);
  }

  /* save current file ptr to restore it if we seek past EOF */
  if ((err == aErrNone) 
      && ((ulOrigFilePtr = ftell(pFile->fp)) == -1))
    err = aErrIO;

  /* get file size to check "sought past EOF" on seek attempt... */
  if ((err == aErrNone) 
      && (aFile_GetSize(libRef, fileRef, (unsigned long*)&ulFileSize, &sizeErr) != aErrNone))
    err = aErrIO;

  /* do some range checking */
  if (err == aErrNone) {
    if (bSeekFromStart == aTrue) {
      lNewPos = lOffset;
    } else {
      lNewPos = ulOrigFilePtr + lOffset;
    }
    if (lNewPos > ulFileSize)
      err = aErrEOF;
    else if (lNewPos < 0)
      err = aErrEOF;
  }

  /* attempt the seek */
  if ((err == aErrNone) 
      && (fseek(pFile->fp, 
  	        lOffset, 
  	        (bSeekFromStart == aTrue) ? SEEK_SET : 
  	        			    SEEK_CUR) != 0)) {
    err = aErrIO;
  }

  /* get current file pointer value after seek */
  if ((err == aErrNone) 
      && ((lFilePtr = ftell(pFile->fp)) == -1))
    err = aErrIO;

  /* Normally if (file ptr == file size), we'd be one byte 
   * past EOF.  But not in the odd case of a zero length file.  
   * Check for that here. */
  if ((err == aErrNone) 
      && ((lFilePtr == 0) 
      && (0 == ulFileSize)))
    err = aErrIO;

  if (pErr != NULL)
    *pErr = err;
  
  return err;

} /* aFile_Seek */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aFile_GetSize
 */

aLIBRETURN 
aFile_GetSize(aIOLib libRef,
	      aFileRef fileRef,
	      unsigned long* pulFileSize,
	      aErr* pErr)
{
  aUnixFile* pFile = NULL;
  aErr err = aErrNone;
  long ulOrigFilePtr = 0;

  aVALIDIO(libRef);
  if ((err == aErrNone)
      && (pulFileSize == NULL))
    err = aErrParam;

  if (err == aErrNone) {
    pFile = (aUnixFile*)fileRef;
    FILECHECK(pFile);
  }

  if (err == aErrNone) {

    *pulFileSize = 0L;

    /* save current file ptr to restore it after seeking to EOF */
    if ((err == aErrNone) 
        && ((ulOrigFilePtr = ftell(pFile->fp)) == -1))
      err = aErrIO;

    /* seek to EOF */
    if ((err == aErrNone) 
        && (fseek(pFile->fp, 0, SEEK_END) != 0))
      err = aErrIO;
    
    /* and read EOF pos as file length */
    if ((err == aErrNone) 
        && ((*pulFileSize = (unsigned long)ftell(pFile->fp)) == (unsigned long)-1)) {
      *pulFileSize = 0;
      err = aErrIO;
    }
    
    /* restore original file ptr pos */
    if ((err == aErrNone) 
        && (fseek(pFile->fp, ulOrigFilePtr, SEEK_SET) != 0))
      err = aErrIO;   
  }

  if (pErr != NULL)
    *pErr = err;
  
  return err;

} /* aFile_GetSize */  				  


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aFile_Delete
 */

aLIBRETURN 
aFile_Delete(aIOLib libRef,
	     const char *pFilename,
	     const aFileArea eArea,
	     aErr* pErr)
{
  aErr err = aErrNone;
  char fullpath[MAXPATHLEN];

  aVALIDIO(libRef);

  if ((err == aErrNone)
      && (pFilename == NULL))
    err = aErrParam;
  else
    unix_aFileFullPath(fullpath, pFilename, eArea);

  /* try to delete and map error */
  if ((err == aErrNone)
      && (unlink(fullpath) != 0)) {
    switch (errno) {
    case ENOENT:
      err = aErrNotFound;
      break;
    default:
      err = aErrIO;
      break;
    } /* switch */
  }
 
  if (pErr != NULL)
    *pErr = err;
  
  return err;

} /* aFile_Delete */

#endif /* aUNIX */

