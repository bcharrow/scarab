/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aUtil.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of platform-independent utilities	   */
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


#ifndef _aUtil_H_
#define _aUtil_H_

#include "aIO.h"

#define aUTIL_EXPORT

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* these routines resolve the endian mess on all platforms */
void aUtil_StoreShort(char* storage, aSHORT val);
void aUtil_StoreInt(char* storage, int val);
void aUtil_StoreFloat(char* storage, float val);
void aUtil_StoreLong(char* storage, long l);
  /*long aLong_Flip(const long l);*/

aSHORT aUtil_RetrieveShort(const char* storage);
int aUtil_RetrieveInt(const char* storage);
float aUtil_RetrieveFloat(const char* storage);
unsigned aSHORT aUtil_RetrieveUShort(const char* storage);

/* filename access routines */
void aString_GetFileRoot(char* root, const char* filename);
aLIBRETURN aUtil_GetFileExtension(char* extension, 
				  const char* filename,
				  aErr* pErr);

/* string routines */
int aString_ParseInt(const char* pString);
aLIBRETURN aUtil_ParseFloat(float* pFloat, const char* pString, aErr* pErr);
void aString_FormatFloat(float fVal, char* pString);
const char* aString_StartsWith(const char* string, 
			       const char* prefix);
const char* aString_CopyToWS(char* copy, 
			     const int nMaxLen,
			     const char* source);
const char* aString_CopyToChar(char* copy, 
			       const char* source, 
			       const char c);
aLIBRETURN aUtil_FormatInetAddr(char* string, 
			           const unsigned long address,
				   aErr* pErr);
aErr aULong_FromInetAddr(unsigned long* pAddr,
			 const char* pString);
const char* aString_GetTokenTypeName(char* string, 
			             const tkType type);

/* routines for retrieving data from symbol tables */
aErr aSymbolTable_GetInt(aIOLib ioRef,
		         aSymbolTableRef paramSyms,
		         const char* pKey,
		         int* pInt);

#ifdef __cplusplus 
}
#endif /* __cplusplus */

#endif /* _aUtil_H_ */
