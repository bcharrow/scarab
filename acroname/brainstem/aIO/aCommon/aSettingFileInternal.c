/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSettingFileInternal.c                                    */
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

#include "aOSDefs.h"
#include "aIOInternal.h"
#include "aSymbolTableInternal.h"
#include "aSettingFileInternal.h"
#include "aUtil.h"


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFileInternal_Create
 */

aErr aSettingFileInternal_Create(aIOLib ioRef,
				 const unsigned int nMaxSettingLen,
			 	 const char* pFileName,
			 	 aSettingFile** ppSettingFile)
{
  aErr settingErr = aErrNone;
  aSettingFile* pSettingFile = NULL;

  if ((ppSettingFile == NULL) || (pFileName == NULL))
    settingErr = aErrParam;

  if (settingErr == aErrNone) {
    pSettingFile = (aSettingFile*)aMemAlloc(sizeof(aSettingFile));
    if (pSettingFile == NULL)
      settingErr = aErrMemory;
  }

  if (settingErr == aErrNone) {
    aBZero(pSettingFile, sizeof(aSettingFile));
    pSettingFile->check = aSETTINGFILECHECK;
    pSettingFile->ioLib = ioRef;
    pSettingFile->nMaxSettingLen = nMaxSettingLen;
    settingErr = aMemPoolInternal_Create((aMemSize)nMaxSettingLen, 8, 
      		    			 &pSettingFile->pSettingPool);
  }

  if (settingErr == aErrNone)
    settingErr = aSymbolTableInternal_Create(
    			&pSettingFile->pSettingTable);

  if (settingErr == aErrNone)
    settingErr = sSettingFile_Read(pSettingFile, pFileName);

  if (settingErr == aErrNone)
    *ppSettingFile = pSettingFile;
  else if (pSettingFile != NULL)
    aSettingFileInternal_Destroy(pSettingFile);

  return settingErr;

} /* aSettingFileInternal_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFileInternal_GetInt
 */

aErr aSettingFileInternal_GetInt(aSettingFile* pSettingFile,
			 	 const char* key, 
			 	 int* pInt,
			 	 const int nDefault)
{
  aErr settingErr = aErrNone;
  char* pValue;

  aVALIDSETTINGFILE(pSettingFile);
  
  if (settingErr == aErrNone)
    settingErr = aSymbolTableInternal_Find(pSettingFile->pSettingTable,
  					   key, (void**)&pValue);

  switch (settingErr) {

  case aErrNone:
    aIntFromString(pInt, pValue);
    break;

  case aErrNotFound:
    *pInt = nDefault;
    settingErr = aErrNone;
    break;

  default:
    break;

  } /* switch */

  return settingErr;

} /* aSettingFileInternal_GetInt */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFileInternal_GetULong
 */

aErr aSettingFileInternal_GetULong(aSettingFile* pSettingFile,
			   	   const char* key, 
			   	   unsigned long* pULong,
			   	   const unsigned long nDefault)
{
  aErr settingErr = aErrNone;
  char* pValue;
  int val;

  aVALIDSETTINGFILE(pSettingFile);
  
  if (settingErr == aErrNone)
    settingErr = aSymbolTableInternal_Find(pSettingFile->pSettingTable,
  		    		 	   key, (void**)&pValue);

  switch (settingErr) {

  case aErrNone:
    aIntFromString(&val, pValue);
    *pULong = (unsigned long)val;
    break;

  case aErrNotFound:
    *pULong = nDefault;
    settingErr = aErrNone;
    break;

  default:
    break;

  } /* switch */

  return settingErr;

} /* aSettingFileInternal_GetULong */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFileInternal_GetFloat
 */

aErr aSettingFileInternal_GetFloat(aSettingFile* pSettingFile,
			   	   const char* key, 
			   	   float* pFloat,
			   	   const float fDefault)
{
  aErr settingErr = aErrNone;
  char* pValue;
  float value = 0.0f;

  aVALIDSETTINGFILE(pSettingFile);
  
  if (settingErr == aErrNone)
    settingErr = aSymbolTableInternal_Find(pSettingFile->pSettingTable,
  			    		   key, (void**)&pValue);

  if (settingErr == aErrNone)
    value = aString_ParseFloat(pValue);

  switch (settingErr) {

  case aErrNone:
    *pFloat = value;
    break;

  case aErrNotFound:
    *pFloat = fDefault;
    settingErr = aErrNone;
    break;

  default:
    break;

  } /* switch */

  return settingErr;

} /* aSettingFileInternal_GetFloat */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFileInternal_GetString
 */

aErr aSettingFileInternal_GetString(aSettingFile* pSettingFile,
			    	    const char* key, 
			    	    char** ppString,
			    	    const char* pDefault)
{
  aErr settingErr = aErrNone;
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

  return settingErr;

} /* aSettingFileInternal_GetString */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFileInternal_GetInetAddr
 */

aErr aSettingFileInternal_GetInetAddr(aSettingFile* pSettingFile,
			    	      const char* key, 
			    	      unsigned long* pInetAddr,
			    	      const unsigned long pDefault)
{
  aErr settingErr = aErrNone;
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

  return settingErr;

} /* aSettingFileInternal_GetInetAddr */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFileInternal_SetKey
 */

aErr aSettingFileInternal_SetKey (
  aSettingFile* pSettingFile,
  const char* pKey, 
  const char* pValue
)
{
  aErr settingErr = aErrNone;
  unsigned int len = 0;
  char* pData;

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
    aStringCopy(pData, pValue);

  if (settingErr == aErrNone)
    settingErr = aSymbolTableInternal_Insert(
    			      pSettingFile->pSettingTable, 
  		      	      pKey, pData, 
  		      	      sSettingDeleteProc, 
  		      	      pSettingFile);
  
  return settingErr;

} /* aSettingFileInternal_SetKey */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFileInternal_Destroy
 */

aErr aSettingFileInternal_Destroy(aSettingFile* pSettingFile)
{
  aErr settingErr = aErrNone;

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

  return settingErr;

} /* aSettingFileInternal_Destroy */
#endif
