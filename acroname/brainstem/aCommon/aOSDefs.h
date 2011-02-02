/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aOSDefs.h                                                 */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: This file defines a number of common OS-Specific   */
/*              routines.  Since these routines are roughly        */
/*              equivalent but often have different syntax on      */
/*              different platforms, we #define all of them here   */
/*              to allow cross-platform coding.  This also allows  */
/*              us to implement a function missing on a specific   */
/*              platform and allows us to #define away routines    */
/*              that don't make any sense for another platform     */
/*              (such as memory handle locking on Windows or       */
/*              Unix).  Finally, this allows us to add additional  */
/*              checks (and overhead) in debug builds to check for */
/*              memory leaks, etc.                                 */
/*                                                                 */
/*              There is a section for each platform we port to.   */
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

#ifndef _aOSDefs_H_
#define _aOSDefs_H_


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Win32 definitions
 */

#ifdef aWIN

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef aDEBUG
  #define aDebugAlert(msg)		MessageBox((HWND)NULL, msg,           \
						   TEXT("Debug"), MB_OK);
#else /* aDEBUG */
  #define aDebugAlert(msg)
#endif /* aDEBUG */


/* safe routines which are available on newer platforms, but not older */
#ifdef _MSC_VER
  #define aStringCopySafe(d, l, s)	strcpy_s((d), (l), (s))
  #define aStringCatSafe(d, l, s)	strcat_s((d), (l), (s))
  #define aSNPRINTF                     sprintf_s
  #define aSNSCANF(format, size, ...)   sscanf_s(format, ##__VA_ARGS__)
#else /* _MSC_VER */
  #define aStringCopySafe(d, l, s)      strcpy(d, s)
  #define aStringCatSafe(d, l, s)	strcat((d), (s))
  #define aSNPRINTF(format, size, ...)	sprintf(format, ##__VA_ARGS__)
  #define aSNSCANF(format, size, ...)   sscanf(format, ##__VA_ARGS__)
#endif /* _MSC_VER */


#define aMemCopy(dst, src, len)		memcpy(dst, src, len)
#define aBZero(buffer, len)		memset(buffer, 0x00, len)
#define aStringCopy(dst, src)		strcpy(dst, src)
#define aStringNCopy(dst, src, n)	strncpy(dst, src, n)
#define aStringCompare(s1, s2)		strcmp(s1, s2)
#define aStringLen(s)			strlen(s)
#define aStringCat(s1, s2)		strcat(s1, s2)
#define aStringFromInt(s, i)		aSNPRINTF(s, sizeof(s), "%d", i)
#define aIntFromString(pi, s)		aSNSCANF(s, sizeof(s), "%d", pi)


#define aMemHandle			void *
#define aMemPtr				void *
#define aMemSize			size_t

#define aByte				char
#define aShort				short
#define aFloat				float
#define aInt32				int

#ifdef aLEAKCHECK
  #include "aMemLeakDebug.h"
#else /* aLEAKCHECK */
  #define aMemAlloc(size)		malloc(size)
  #define aMemFree(p)			free(p)
#endif /* aLEAKCHECK */

#define aszEOL				"\x0D\x0A"

#define aLIBRETURN			int
#define aLIBREF				void *
#define aLIB_IMPORT                     __declspec(dllimport)

#define aLITTLEENDIAN

#endif /* aWIN */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * UNIX definitions (all Unix platforms and MacOS X)
 */

#if defined(aUNIX) || defined(aMACX)

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef aDEBUG
#ifdef aMACX
#define aDebugAlert(msg)		printf("ERROR! %s\n", msg)
#else
#define aDebugAlert(msg)		printf("ERROR! %s\n", msg)
#endif
#else
#define aDebugAlert(msg)
#endif

#define aMemCopy(dst, src, len)		memcpy(dst, src, len)
#define aBZero(buffer, len)		memset(buffer, 0x00, len)
#define aStringCopy(dst, src)		strcpy(dst, src)
#define aStringNCopy(dst, src, n)	strncpy(dst, src, n)
#define aStringCompare(s1, s2)		strcmp(s1, s2)
#define aStringLen(s)			strlen(s)
#define aStringCat(s1, s2)		strcat(s1, s2)
#define aStringFromInt(s, i)		snprintf(s, sizeof(s), "%d", (int)i)
#define aIntFromString(pi, s)		sscanf(s, "%d", pi)

/* safe routines */
#ifndef strlcpy
#ifdef __cplusplus
extern "C" {
#endif
size_t strlcpy(char* dst, const char* src, size_t len);
#ifdef __cplusplus
}
#endif
#endif
#ifndef strlcat
#ifdef __cplusplus
extern "C" {
#endif
size_t strlcat(char *dst, const char *src, size_t siz);
#ifdef __cplusplus
}
#endif
#endif

#define aStringCopySafe(d, l, s)	strlcpy((d), (s), (l))
#define aStringNCopySafe(d, l, s)	strlcpy((d), (s), (l))
#define aStringCatSafe(d, l, s)	        strlcat((d), (s), (l))
#define aSNPRINTF                       snprintf
#define aSNSCANF(format, size, ...)     sscanf(format, ##__VA_ARGS__)

#define aByte				char
#define aShort				short
#define aFloat				float
#define aInt32				int

#ifdef aMACX
#define aMemHandle			char**
#else
#define aMemHandle			void *
#endif
#define aMemPtr				void *
#define aMemSize			size_t

#ifdef aLEAKCHECK
#include "aMemLeakDebug.h"
#else /* aLEAKCHECK */
#define aMemAlloc(size)			malloc(size)
#define aMemFree(p)			free(p)
#endif /* aLEAKCHECK */

#define aszEOL				"\n"

#define aLIBRETURN			int
#define aLIBREF				void *
#define aLIB_IMPORT

#ifdef aMACX 

 #ifdef __BIG_ENDIAN__
  #define aBIGENDIAN
 #elif __LITTLE_ENDIAN__
  #define aLITTLEENDIAN
 #endif

#else /* else not mac so we are likely little endian */
 #define aLITTLEENDIAN
#endif

#endif /* aUNIX */





/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Common word order handling based on aBIGENDIAN or aLITTLEENDIAN
 * defines created above
 */

#ifdef aBIGENDIAN    /* network word order */
 #define aH2TS(s)			s
 #define aT2HS(s)			s
 #define aH2TUS(s)			s
 #define aT2HUS(s)			s
 #define aH2TA(s)			s
 #define aT2HA(s)			s
#else
 #ifdef aLITTLEENDIAN
  #define aH2TS(s)  (aSHORT)(((s & 0x00FF) << 8) | ((s & 0xFF00) >> 8))
  #define aT2HS(s)  (aSHORT)(((s & 0x00FF) << 8) | ((s & 0xFF00) >> 8))
  #define aH2TUS(s) (unsigned aSHORT)(((s & 0x00FF) << 8) | ((s & 0xFF00) >> 8))
  #define aT2HUS(s) (unsigned aSHORT)(((s & 0x00FF) << 8) | ((s & 0xFF00) >> 8))
  #define aH2TA(s)  (tADDRESS)(((s & 0x00FF) << 8) | ((s & 0xFF00) >> 8))
  #define aT2HA(s)  (tADDRESS)(((s & 0x00FF) << 8) | ((s & 0xFF00) >> 8))
 #else
   error!!!! Word Order Not Defined and likely no prefix header
 #endif
#endif 


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Common piece for aAssert implementation
 */

#ifdef aDEBUG
#include "aAssert.h"
#else
#define aAssert(exp)
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Maximum number of filename bytes.
 */

#define aFILE_NAMEMAXCHARS	31

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * We define a unique boolean type to avoid conflict with other 
 * software the libraries or software may interact with.
 */
typedef int aBool;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * These seem obvious but it can be quite useful to flush out 
 * bugs to use these instead of 1 and 0 directly.
 */
#define aTrue 1
#define aFalse 0

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * some compilers don't define NULL so we do it here
 */
#ifndef NULL
#define NULL 0L
#endif /* NULL */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * signature in front of all compiled tea files
 */
#define aTEA_4BYTE_SIGNATURE	"aTEA"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * math stuff
 */
#define aPI 3.1415926535897932384626433832795
#define a2PI 6.283185307179586476925286766559

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * basic types
 */
#define aSHORT	short
#define aFLOAT	float

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * version packing to uniformly handle major, minor, and builds
 */

#define aVERSION_PACK(maj, min, bld) (((maj) << 28)                  \
				      | ((min) << 24)                \
				      | ((bld) & 0xFFFFFF))
#define aVERSION_UNPACK_MAJOR(pack) ((pack) >> 28)
#define aVERSION_UNPACK_MINOR(pack) ((pack & 0xF000000) >> 24)
#define aVERSION_UNPACK_BUILD(pack) ((pack) & 0xFFFFFF)

#endif /* _aOSDefs_H_ */
