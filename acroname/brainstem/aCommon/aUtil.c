/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aUtil.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent utilities */
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
#include "aUtil.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUtil_StoreShort
 */

void aUtil_StoreShort(char* storage, aSHORT val)
{
  aSHORT temp = aH2TS(val);
  aMemCopy(storage, &temp, sizeof(aSHORT));

} /* aUtil_StoreShort */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUtil_StoreInt
 */

void aUtil_StoreInt(char* storage, int nVal)
{
#ifdef aBIGENDIAN
  aMemCopy(storage, &nVal, sizeof(int));
#else
  char* data = (char*)&nVal;

  union {
    char  c[sizeof(int)];
    int i;
  } tmp;
  register int i;
  for (i = sizeof(int); i--; ) 
    tmp.c[(sizeof(int)-1) - i] = data[i];

  aMemCopy(storage, &tmp.i, sizeof(int));
#endif
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUtil_StoreFloat
 */

void aUtil_StoreFloat(char* storage, float fVal)
{
#ifdef aBIGENDIAN
  aMemCopy(storage, &fVal, sizeof(float));
#else
  char* data = (char*)&fVal;

  union {
    char  c[sizeof(float)];
    int i;
  } tmp;
  register int i;
  for (i = sizeof(float); i--; ) 
    tmp.c[(sizeof(float)-1) - i] = data[i];

  aMemCopy(storage, &tmp.i, sizeof(float));
#endif
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUtil_StoreLong
 */

void aUtil_StoreLong(char* storage, long l)
{
#ifdef aBIGENDIAN
  aMemCopy(storage, &l, sizeof(long));
#else
  char* data = (char*)&l;

  union {
    char  c[sizeof(long)];
    long l;
  } tmp;
  register int i;
  for (i = sizeof(long); i--; ) 
    tmp.c[(sizeof(long)-1) - i] = data[i];

  aMemCopy(storage, &tmp.l, sizeof(long));
#endif

} /* aUtil_StoreLong */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUtil_RetrieveShort
 */

aSHORT aUtil_RetrieveShort(const char* storage)
{
  aSHORT temp;
  aSHORT temp1;

  aMemCopy(&temp, storage, sizeof(aSHORT));
  temp1 = aT2HS(temp);
  return temp1;

} /* aUtil_RetrieveShort */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUtil_RetrieveInt
 */

int aUtil_RetrieveInt(const char* storage)
{
  int val;

#ifdef aBIGENDIAN
  aMemCopy((char*)&val, storage, sizeof(int));
#else
  const char* data = storage;

  union {
    char  c[sizeof(int)];
    int i;
  } tmp;
  register int i;
  for (i=sizeof(int); i--; ) 
    tmp.c[(sizeof(float) - 1) - i] = data[i];

  aMemCopy((char*)&val, &tmp.i, sizeof(int));
#endif

  return val;

} /* aUtil_RetrieveInt */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUtil_RetrieveFloat
 */

float aUtil_RetrieveFloat(const char* storage)
{
  float val;

#ifdef aBIGENDIAN
  aMemCopy((char*)&val, storage, sizeof(float));
#else
  const char* data = storage;

  union {
    char  c[sizeof(float)];
    float f;
  } tmp;
  register int i;
  for (i=sizeof(float); i--; ) 
    tmp.c[(sizeof(float) - 1) - i] = data[i];

  aMemCopy((char*)&val, &tmp.f, sizeof(float));
#endif

  return val;

} /* aUtil_RetrieveFloat */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUtil_RetrieveUShort
 */

unsigned short aUtil_RetrieveUShort(const char* storage)
{
  unsigned short temp;
  unsigned short temp1;

  aMemCopy(&temp, storage, sizeof(unsigned short));
  temp1 = aT2HUS(temp);
  return temp1;

} /* aUtil_RetrieveUShort */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aString_GetFileRoot
 *
 *   root must be at least aFILE_NAMEMAXCHARS
 */

void aString_GetFileRoot(char* root, 
			 const char* filename)
{
  unsigned int len = (unsigned int)aStringLen(filename);
  char* p = (char*)&filename[len-1];
  
  while (p != filename) {
    if (*p == '.')
      break;
    p--;
    len--;
  }

  if (len > 1) {
    len--;
    aStringCopySafe(root, aFILE_NAMEMAXCHARS, filename);
    root[len] = 0;
  } else {
    aStringCopySafe(root, aFILE_NAMEMAXCHARS, filename);
  }

} /* aString_GetFileRoot */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aString_GetFileExtension
 */

aLIBRETURN
aUtil_GetFileExtension(char* extension, 
		       const char* filename,
		       aErr* pErr)
{
  aErr err = aErrNone;
  unsigned int len = (unsigned int)aStringLen(filename);
  char* p = (char*)&filename[len-1];
  
  while (p != filename) {
    if (*p == '.')
      break;
    p--;
    len--;
  }

  if (p == filename)
    extension[0] = 0;
  else {
    aStringCopySafe(extension, aFILE_NAMEMAXCHARS, p);
  }
  
  if (pErr)
    *pErr = err;
  
  return (aLIBRETURN)(err != aErrNone);

} /* aString_GetFileExtension */


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aString_GetFileExtension
 */

void aString_SetNumParam(char* result, 
			 const char* source, 
			 int num)
{
  char* p = (char*)source;
  char* d = (char*)result;
  char numStr[10];

  while (*p) {
    if (*p == '^') {
      char* n = numStr;
      p += 2;
      aStringFromInt(numStr, num);
      while (*n)
        *d++ = *n++;
    } else {
      *d++ = *p++;
    }
  }

  /* null terminate */
  *d = 0;

} /* aString_SetNumParam */
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aString_ParseInt
 *
 * Parses an integer in either decimal or hexidecimal format.
 */

int 
aString_ParseInt(const char* pString)
{
  const char* p = pString;
  int b = 10;
  int v = 0;
  aBool bHex = aFalse;
  aBool bNegative = aFalse;

  while (*p) {
    if ((*p >= '0') && (*p <= '9')) {
      if (v) v *= b;
      v += *p - '0';
    } else if (bHex && ((*p >= 'A') && (*p <= 'F'))) {
      if (v) v *= b;
      v += *p - 'A' + 10;
    } else if (bHex && ((*p >= 'a') && (*p <= 'f'))) {
      if (v) v *= b;
      v += *p - 'a' + 10;
    } else if (*p == '-') {
      if (v || bNegative)
	goto err;
      bNegative = aTrue;
    } else if (*p == 'x') {
      if (bHex) /* error as we have already seen an x */
	goto err;
      bHex = aTrue;
      b = 16;
    } else {
      goto err;
    }
    p++;
  }
  
  if (bNegative)
    v = -v;

  return v;

err:
  return 0;

} /* aString_ParseInt */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aString_ParseFloat
 */

aLIBRETURN 
aUtil_ParseFloat(float* pFloat, const char* pString, aErr* pErr)
{
  aErr err = aErrNone;
  const char* p = pString;
  float fDivisor = 0.1f;
  int nNum;
  aBool bDecimal = aFalse;
  aBool bNegative = aFalse;
  float fValue = 0.0f;
  
  /* parse the float */
  while ((err == aErrNone) && *p) {
    
    /* record the decimal occurance and error if already there */
    if (*p == '.') {
      if (bDecimal == aTrue)
	err = aErrParam;
      else
	bDecimal = aTrue;
      
    } else if (*p == '-') {
      if (bNegative == aTrue)
	err = aErrParam;
      else
	bNegative = aTrue;
      
      /* just bail on bad characters */
    } else if ((*p < '0') || (*p > '9')) {
      err = aErrParam;;
      
      /* accumulate the value */
    } else {
      nNum = *p - '0';
      if (bDecimal == aFalse) {
        fValue *= 10;
        fValue += nNum;
      } else {
        fValue += (float)nNum * fDivisor;
        fDivisor /= 10.0f;
      }
    }
    p++;
  }
  
  if (bNegative)
    fValue = -fValue;
  
  if (err == aErrNone)
    *pFloat = fValue;
  
  if (pErr)
    *pErr = err;
  
  return (aLIBRETURN)(err != aErrNone);

} /* aString_ParseFloat */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aString_FormatFloat
 */

void aString_FormatFloat(float fVal, char* pString)
{
  aBool bNegative = (fVal < 0) ? aTrue : aFalse;
  int nNum = (int)fVal;
  float fFraction = fVal - nNum;
  char num[10];
  size_t buflen = 10; /* this entire routine needs to be replaced by acpString */

  *pString = 0;
  if (bNegative) {
    aStringCatSafe(pString, buflen, "-");
    fFraction = -fFraction;
    nNum = -nNum;
  }
  aStringFromInt(num, nNum);
  aStringCatSafe(pString, buflen, num);
  aStringCatSafe(pString, buflen, ".");
  nNum = (int)(1000.0f * fFraction);
  if (nNum < 100)
    aStringCatSafe(pString, buflen, "0");
  if (nNum < 10)
    aStringCatSafe(pString, buflen, "0");
  aStringFromInt(num, nNum);
  aStringCatSafe(pString, buflen, num);

} /* aString_FormatFloat */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aString_StartsWith
 * 
 * Returns first character after the prefix if found, null if not.
 */

const char* aString_StartsWith(const char* string, 
			       const char* prefix)
{
  const char* s = string;
  const char* p = prefix;

  while (*p && *s) {
    if (*p != *s)
      return (char*)NULL;
    p++;
    s++;
  }
  if (!*p)
    return s;
  
  return (char*)NULL;

} /* aString_StartsWith */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aString_CopyToWS
 */

const char* aString_CopyToWS(char* copy, 
			     const int nMaxLen,
			     const char* source)
{
  char* c = copy;
  const char* s = source;
  int len = 0;

  while (*s && (*s != ' ') && (*s != '\t') && (++len < nMaxLen))
    *c++ = *s++;
  
  *c = 0;
  
  return s;

} /* aString_CopyToWS */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aString_CopyToChar
 */

const char* aString_CopyToChar(char* copy, 
		               const char* source, 
		               const char c)
{
  char* cp = copy;
  const char* s = source;
  while (*s && (*s != c))
    *cp++ = *s++;

  *cp = 0;
  
  if (*s == c)
    s++;

  return s;

} /* aString_CopyToChar */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aString_FormatInetAddr
 *
 * Formats an IP address into the common DDD.DDD.DDD.DDD format.
 *
 * string passed in must have at least 16 bytes (including
 * terminator).
 */

aLIBRETURN
aUtil_FormatInetAddr(
  char* string, 
  const unsigned long address,
  aErr* pErr)
{
  aErr err = aErrNone;
  char* p = string;
  aStringFromInt(p, (int)((address & 0xFF000000) >> 24));
  p += aStringLen(p);
  *p++ = '.';
  aStringFromInt(p, (int)((address & 0xFF0000) >> 16));
  p += aStringLen(p);
  *p++ = '.';
  aStringFromInt(p, (int)((address & 0xFF00) >> 8));
  p += aStringLen(p);
  *p++ = '.';
  aStringFromInt(p, (int)(address & 0xFF));

  if (pErr)
    *pErr = err;

  return (aLIBRETURN)(err == aErrNone);

} /* aString_FormatInetAddr */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aULong_FromInetAddr
 *
 * Parses an IP address from the common DDD.DDD.DDD.DDD format.
 *
 * string passed in must have at least 16 bytes (including
 * terminator).
 */

aErr aULong_FromInetAddr(
  unsigned long* pAddr,
  const char* pString)
{
  aErr err = aErrNone;
  unsigned long ul = 0;
  const char* p = pString;
  int byte = 0;
  int num = 0;
  int shift = 24;

  /* parse the string */
  while (*p && (err == aErrNone)) {
    if (*p == '.') {
      if (shift < 8)
	err = aErrParse;
      else {
	ul += (unsigned long)(byte << shift);
	shift -= 8;;
	byte = 0;
      }
    } else if ((*p < '0') || (*p > '9')) {
      err = aErrParse;
    } else {
      num = *p - '0';
      byte *= 10;
      byte += num;
      if (byte > 255)
	err = aErrParse;
    }
    p++;
  }

  if (err == aErrNone) {
    ul += (unsigned long)byte;
    *pAddr = ul;
  }

  return err;

} /* aULong_FromInetAddr */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aString_GetTokenTypeName
 *
 * Returns a text version of the token type.
 */

const char* aString_GetTokenTypeName(char* string, 
			             const tkType type)
{
  switch (type) {
  case tkInt:
    aStringCopySafe(string, 24, "integer");
    break;
  case tkFloat:
    aStringCopySafe(string, 24, "float");
    break;
  case tkIdentifier:
    aStringCopySafe(string, 24, "identifier");
    break;
  case tkSpecial:
    aStringCopySafe(string, 24, "special character");
    break;
  case tkString:
    aStringCopySafe(string, 24, "string");
    break;
  case tkPreProc:
    aStringCopySafe(string, 24, "pre-processor directive");
    break;
  case tkNewLine:
    aStringCopySafe(string, 24, "newline");
    break;
  } /* switch */

  return string;

} /* aString_GetTokenTypeName */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSymbolTable_GetInt
 */

aErr aSymbolTable_GetInt(aIOLib ioRef,
		         aSymbolTableRef paramSyms,
		         const char* pKey,
		         int* pInt)
{
  aErr err = aErrNone;
  char* pIntStr = NULL;

  if ((err == aErrNone) && (!pKey || !pInt))
    err = aErrParam;
  
  if (err == aErrNone) {
    /* this will return aErrNotFound if the param is not present */
    void* p = (void*)pIntStr;
    aSymbolTable_Find(ioRef, paramSyms, pKey, &p, &err);
    if (err == aErrNone)
      aIntFromString(pInt, pIntStr);
  }
  
  return err;

} /* aSymbolTable_GetInt */
