/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSettingFile.c						   */
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

#include "aUtil.h"
#include "aIOInternal.h"
#include "aMemPoolInternal.h"
#include "aSymbolTableInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local defines
 */

#define aSETTINGLINEBUFMAX	100

typedef struct aSettingFile {
  aIOLib		ioLib;
  unsigned int		nMaxSettingLen;
  aMemPool*		pSettingPool;
  aSymbolTable*		pSettingTable;
  char			readBuf[aSETTINGLINEBUFMAX];
  int			check;
} aSettingFile;

#define aSETTINGFILECHECK	0xDDDD

#define aVALIDSETTINGFILE(p)					   \
  if ((p == NULL) ||						   \
      (((aSettingFile*)p)->check != aSETTINGFILECHECK)) {   	   \
    settingErr = aErrParam;					   \
  }


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static void sSettingFile_ReadLine(aSettingFile* pSettings);
static aErr sSettingFile_Read(aSettingFile* pSettings,
			      const char* pFileName);
static aErr sSettingDeleteProc(void* pData, void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSettingDeleteProc
 */

aErr 
sSettingDeleteProc(void* pData, void* ref)
{
  aSettingFile* pSettings = (aSettingFile*)ref;

  if (!pData || !pSettings)
    return aErrParam;

  return aMemPoolInternal_Free(pSettings->pSettingPool, pData);

} /* sSettingDeleteProc */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSettingFile_ReadLine
 */

void 
sSettingFile_ReadLine(aSettingFile* pSettingFile)
{
  aErr e;
  char* p = pSettingFile->readBuf;
  char key[100];
  char* k = key;
  char value[100];
  char* v = value;
  char* pData;
  unsigned int len = 0;
    
  /* skip over initial whitespace */
  while (*p && ((*p == ' ') || (*p == '\t'))) p++;

  /* ignore comments starting with "#" */  
  if (*p == '#')
    return;

  /* move up to the equal sign */
  while (*p && ((*p != ' ') && (*p != '\t') && (*p != '='))) {
    *k++ = *p++;
  }

  /* null terminate the key */
  *k = 0;
  
  if (!*p)
    return;

  /* skip over the equal and any whitespace */
  while (*p && ((*p == ' ') || (*p == '\t') || (*p == '='))) p++;

  if (!*p)
    return;

  /* move up to the end of line */
  while (*p) {
    *v++ = *p++;
    len++;
  }

  /* now, chomp off any whitespace at the end of the string */
  do {
    v--;
  } while ((v != value) && ((*v == ' ') || (*v == '\t')));
  v++;

  /* null terminate the value */
  *v = 0;
  
  /* see if the value is already in place */
  

  /* get storage for the value from the memory pool */
  if (aMemPoolInternal_Alloc(pSettingFile->pSettingPool, 
  			     (void**)&pData))
    return;

  if (len > pSettingFile->nMaxSettingLen) {
    aMemCopy(pData, value, pSettingFile->nMaxSettingLen - 1);
    pData[pSettingFile->nMaxSettingLen - 1] = 0;
  } else
    aStringCopySafe(pData, pSettingFile->nMaxSettingLen, value);

  e = aSymbolTableInternal_Insert(pSettingFile->pSettingTable, 
				  key, pData, sSettingDeleteProc, 
				  pSettingFile);
  if (e == aErrDuplicate)
    aMemPoolInternal_Free(pSettingFile->pSettingPool, pData);

} /* sSettingFile_ReadLine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSettingFile_Read
 */

aErr 
sSettingFile_Read(aSettingFile* pSettings,
		  const char* pFileName)
{
  aErr settingErr = aErrNone;
  aErr ioErr = aErrNone;
  aStreamRef file = NULL;

  /* open the file stream */
  if (settingErr == aErrNone) {
    if (aStream_CreateFileInput(pSettings->ioLib, 
    				pFileName, aFileAreaBinary,  
    		   		&file, &ioErr))
    settingErr = aErrNotFound;
  }

  /* read in the file */  
  if (settingErr == aErrNone) {
    do {
      aStream_ReadLine(pSettings->ioLib, file, pSettings->readBuf, 
      		       aSETTINGLINEBUFMAX, &ioErr);
      if (ioErr == aErrNone) {
        /* we now have a line, parse it and store in the 
         * symbol table */
        sSettingFile_ReadLine(pSettings);
      }
    } while (ioErr == aErrNone);
  }

  /* close the file stream */
  if (file != NULL)
    aStream_Destroy(pSettings->ioLib, file, NULL);

  /* just fall back on defaults if file not found */
  if (settingErr == aErrNotFound)
    settingErr = aErrNone;

  return settingErr;

} /* sSettingFile_Read */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFile_Create
 */

aLIBRETURN 
aSettingFile_Create(aIOLib ioRef,
		    const unsigned int nMaxSettingLen,
		    const char* pFileName,
		    aSettingFileRef* pSettingFileRef,
		    aErr* pErr)
{
  aErr err = aErrNone;
  aSettingFile* pSettingFile = NULL;

  aVALIDIO(ioRef);
  if ((pSettingFileRef == NULL) || (pFileName == NULL))
    err = aErrParam;

  if (err == aErrNone) {
    pSettingFile = (aSettingFile*)aMemAlloc(sizeof(aSettingFile));
    if (pSettingFile == NULL)
      err = aErrMemory;
  }

  if (err == aErrNone) {
    aBZero(pSettingFile, sizeof(aSettingFile));
    pSettingFile->check = aSETTINGFILECHECK;
    pSettingFile->ioLib = ioRef;
    pSettingFile->nMaxSettingLen = nMaxSettingLen;
    err = aMemPoolInternal_Create((aMemSize)nMaxSettingLen, 8, 
      		    			 &pSettingFile->pSettingPool);
  }

  if (err == aErrNone)
    err = aSymbolTableInternal_Create(
    			&pSettingFile->pSettingTable);

  if (err == aErrNone)
    err = sSettingFile_Read(pSettingFile, pFileName);

  if (err == aErrNone)
    *pSettingFileRef = (void*)pSettingFile;
  else if (pSettingFile != NULL)
    aSettingFile_Destroy(ioRef, (void*)pSettingFile, NULL);

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aSettingFile_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFile_GetInt
 */

aLIBRETURN 
aSettingFile_GetInt(aIOLib ioRef,
		    aSettingFileRef settingFileRef,
		    const char* key, 
		    int* pInt,
		    const int nDefault,
		    aErr* pErr)
{
  aErr settingErr = aErrNone;
  aSettingFile* pSettingFile = (aSettingFile*)settingFileRef;
  char* pValue;

  aVALIDSETTINGFILE(pSettingFile);
  
  if (settingErr == aErrNone)
    settingErr = aSymbolTableInternal_Find(pSettingFile->pSettingTable,
  					   key, (void**)&pValue);

  switch (settingErr) {

  case aErrNone:
    *pInt = aString_ParseInt(pValue);
    break;

  case aErrNotFound:
    *pInt = nDefault;
    settingErr = aErrNone;
    break;

  default:
    break;

  } /* switch */

  if (pErr != NULL)
    *pErr = settingErr;

  return (aLIBRETURN)(settingErr != aErrNone);

} /* aSettingFile_GetInt */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFile_GetULong
 */
			 
aLIBRETURN 
aSettingFile_GetULong(aIOLib ioRef,
		      aSettingFileRef settingFileRef,
		      const char* key, 
		      unsigned long* pULong,
		      const unsigned long nDefault,
		      aErr* pErr)
{
  aErr settingErr = aErrNone;
  aSettingFile* pSettingFile = (aSettingFile*)settingFileRef;
  char* pValue;

  aVALIDSETTINGFILE(pSettingFile);
  
  if (settingErr == aErrNone)
    settingErr = aSymbolTableInternal_Find(pSettingFile->pSettingTable,
  		    		 	   key, (void**)&pValue);

  switch (settingErr) {

  case aErrNone:
    if (aSNSCANF(pValue, pSettingFile->nMaxSettingLen, "%lu", pULong) != 1)
      settingErr = aErrParam;
    break;

  case aErrNotFound:
    *pULong = nDefault;
    settingErr = aErrNone;
    break;

  default:
    break;

  } /* switch */

  if (pErr != NULL)
    *pErr = settingErr;

  return (aLIBRETURN)(settingErr != aErrNone);

} /* aSettingFile_GetULong */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFile_GetFloat
 */
		   
aLIBRETURN 
aSettingFile_GetFloat(aIOLib ioRef,
		      aSettingFileRef settingFileRef,
		      const char* key, 
		      float* pFloat,
		      const float fDefault,
		      aErr* pErr)
{
  aErr settingErr = aErrNone;
  aSettingFile* pSettingFile = (aSettingFile*)settingFileRef;
  char* pValue;

  aVALIDSETTINGFILE(pSettingFile);

  if (settingErr == aErrNone)
    settingErr = aSymbolTableInternal_Find(pSettingFile->pSettingTable,
  			    		   key, (void**)&pValue);

  switch (settingErr) {

  case aErrNone:
    aUtil_ParseFloat(pFloat, pValue, NULL);
    break;

  case aErrNotFound:
    *pFloat = fDefault;
    settingErr = aErrNone;
    break;

  default:
    break;

  } /* switch */

  if (pErr != NULL)
    *pErr = settingErr;

  return (aLIBRETURN)(settingErr != aErrNone);

} /* aSettingFile_GetFloat */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFile_GetString
 */

aLIBRETURN 
aSettingFile_GetString(aIOLib ioRef,
		       aSettingFileRef settingFileRef,
		       const char* key, 
		       char** ppString,
		       const char* pDefault,
		       aErr* pErr)
{
  aErr settingErr = aErrNone;
  aSettingFile* pSettingFile = (aSettingFile*)settingFileRef;
  void* pValue;

  aVALIDSETTINGFILE(pSettingFile);
  
  if (settingErr == aErrNone)
    settingErr = aSymbolTableInternal_Find(pSettingFile->pSettingTable,
  			    		   key, (void**)&pValue);

  switch (settingErr) {

  case aErrNone:
    *ppString = (char*)pValue;
    break;

  case aErrNotFound:
    *ppString = (char*)pDefault;
    settingErr = aErrNone;
    break;

  default:
    break;

  } /* switch */

  if (pErr != NULL)
    *pErr = settingErr;

  return (aLIBRETURN)(settingErr != aErrNone);

} /* aSettingFile_GetString */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFile_GetInetAddr
 */

aLIBRETURN aSettingFile_GetInetAddr(aIOLib ioRef,
				    aSettingFileRef settingFileRef,
			    	    const char* key, 
			    	    unsigned long* pInetAddr,
			    	    const unsigned long pDefault,
			  	    aErr* pErr)
{
  aErr settingErr = aErrNone;
  aSettingFile* pSettingFile = (aSettingFile*)settingFileRef;
  char* pValue;

  aVALIDSETTINGFILE(pSettingFile);

  if ((settingErr == aErrNone) && !pInetAddr)
    settingErr = aErrParam;
  
  if (settingErr == aErrNone)
    settingErr = aSymbolTableInternal_Find(pSettingFile->pSettingTable,
  			    		   key, (void**)&pValue);

  switch (settingErr) {

  case aErrNone:
    settingErr = aULong_FromInetAddr(pInetAddr, pValue);
    break;

  case aErrNotFound:
    *pInetAddr = pDefault;
    settingErr = aErrNone;
    break;

  default:
    break;

  } /* switch */

  if (pErr != NULL)
    *pErr = settingErr;

  return (aLIBRETURN)(settingErr != aErrNone);

} /* aSettingFile_GetInetAddr */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFile_SetKey
 */

aLIBRETURN 
aSettingFile_SetKey(aIOLib ioRef,
		    aSettingFileRef settingFileRef,
		    const char* pKey, 
		    const char* pValue,
		    aErr* pErr)
{
  aErr settingErr = aErrNone;
  unsigned int len = 0;
  char* pData;
  aSettingFile* pSettingFile = (aSettingFile*)settingFileRef;

  aVALIDSETTINGFILE(pSettingFile);

  if (settingErr == aErrNone) {
    len = aStringLen(pValue) + 1;
    if (len > pSettingFile->nMaxSettingLen)
      settingErr = aErrRange;
  } 

  /* get storage for the value from the memory pool */
  if (settingErr == aErrNone)
    settingErr = aMemPoolInternal_Alloc(pSettingFile->pSettingPool, 
  			     		(void**)&pData);

  if (settingErr == aErrNone)
    aStringCopySafe(pData, pSettingFile->nMaxSettingLen, pValue);

  if (settingErr == aErrNone)
    settingErr = aSymbolTableInternal_Insert(
    			      pSettingFile->pSettingTable, 
  		      	      pKey, pData, 
  		      	      sSettingDeleteProc, 
  		      	      pSettingFile);
  
  if (pErr != NULL)
    *pErr = settingErr;
  
  return (aLIBRETURN)(settingErr != aErrNone);
  
} /* aSettingFile_SetKey */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFile_AddArguments
 */

aLIBRETURN 
aSettingFile_AddArguments(aIOLib ioRef,
			  aSettingFileRef settingFileRef,
			  const int argc, 
			  const char* argv[],
			  aErr* pErr)
{
  aErr err = aErrNone;
  int i;

  /* now, we scan for arguments with one of the following two
   *formats and add them to the keys in the file
   * -g -k
   * -g value
   */
  for (i = 0; (err == aErrNone) && (i < argc); i++) {
    const char* pKey = argv[i];
    if (pKey && (*pKey == '-')) {
      char* pVal = "";
      pKey++; /* skip the '-' */
      if ((i + 1) < argc) {
	const char* pNext = argv[i+1];
	if (pNext && (*pNext != '-')) {
	  pVal = (char*)pNext;
	  i++;
	}
      }
      aSettingFile_SetKey(ioRef, settingFileRef, pKey, pVal, &err);
    }
  }
  
  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);
  
} /* aSettingFile_AddArguments */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFile_Destroy
 */

aLIBRETURN aSettingFile_Destroy(aIOLib ioRef,
				aSettingFileRef settingFileRef,
			  	aErr* pErr)
{
  aErr settingErr = aErrNone;
  aSettingFile* pSettingFile = (aSettingFile*)settingFileRef;

  aVALIDSETTINGFILE(pSettingFile);

  if (settingErr == aErrNone) {
    if (pSettingFile->pSettingTable != NULL) {
      aSymbolTableInternal_Destroy(pSettingFile->pSettingTable);
      pSettingFile->pSettingTable = NULL;
    }
    if (pSettingFile->pSettingPool != NULL) {
      aMemPoolInternal_Destroy(pSettingFile->pSettingPool);
      pSettingFile->pSettingPool = NULL;
    }
    pSettingFile->ioLib = NULL;
    pSettingFile->check = 0;
    aMemFree((aMemPtr)pSettingFile);
  }

  if (pErr != NULL)
    *pErr = settingErr;

  return (aLIBRETURN)(settingErr != aErrNone);

} /* aSettingFile_Destroy */
