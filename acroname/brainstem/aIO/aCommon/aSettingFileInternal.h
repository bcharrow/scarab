/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSettingFileInternal.h                                    */
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

#ifndef _aSettingFileInternal_H_
#define _aSettingFileInternal_H_

#if 0
#include "aIOInternal.h"
#include "aMemPoolInternal.h"
#include "aSymbolTableInternal.h"

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

aErr 
aSettingFileInternal_Create(aIOLib ioRef,
			    const unsigned int nMaxSettingLen,
			    const char* pFileName,
			    aSettingFile** ppSettingFile);

aErr 
aSettingFileInternal_GetInt(aSettingFile* pSettingFile,
			    const char* key, 			    
			    int* pInt,
			    const int nDefault);

aErr 
aSettingFileInternal_GetULong(aSettingFile* pSettingFile,
			      const char* key, 
			      unsigned long* pULong,
			      const unsigned long nDefault);

aErr 
aSettingFileInternal_GetFloat(aSettingFile* pSettingFile,
			      const char* key, 
			      float* pFloat,
			      const float fDefault);

aErr 
aSettingFileInternal_GetString(aSettingFile* pSettingFile,
			       const char* key, 
			       char** ppString,
			       const char* pDefault);

aErr 
aSettingFileInternal_GetInetAddr(aSettingFile* pSettingFile,
				 const char* key, 
				 unsigned long* pInetAddr,
				 const unsigned long pDefault);

aErr 
aSettingFileInternal_SetKey(aSettingFile* pSettingFile,
			    const char* pKey, 
			    const char* pValue);

aErr 
aSettingFileInternal_Destroy(aSettingFile* pSettingFile);

#endif

#endif /* _aSettingFileInternal_H_ */
