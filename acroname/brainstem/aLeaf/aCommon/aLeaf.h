/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aLeaf.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: export definitions for win32 DLL.                  */
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

#ifndef _aLeaf_H_
#define _aLeaf_H_

#include "aIO.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * opaque reference to the compiler library
 */

typedef aLIBREF aLeafLib;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * define symbol import mechanism
 */

#ifndef aLEAF_EXPORT
#define aLEAF_EXPORT aLIB_IMPORT
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * compiler flags
 */

#define	fLeafPreprocess			0x01
#define fLeafAST	 		0x02
#define fLeafGenerateCode 		0x04
#define fLeafDisassemble		0x08


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * compiler library manipulation routines
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

aLEAF_EXPORT aLIBRETURN 
aLeaf_GetLibRef(aLeafLib* pLeafRef, 
		aErr* pErr);

aLEAF_EXPORT aLIBRETURN 
aLeaf_ReleaseLibRef(aLeafLib leafRef, 
		   aErr* pErr);

aLEAF_EXPORT aLIBRETURN 
aLeaf_GetVersion(aLeafLib leafRef,
		 unsigned long* pVersion, 
		 aErr* pErr);

#ifdef __cplusplus
}
#endif /* __cplusplus */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * compiler routines
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

aLEAF_EXPORT aLIBRETURN
aLeaf_Compile(aLeafLib leafRef,
	      aStreamRef source,
	      const char* sourceName,
	      int flags,
	      aStreamRef result,
	      aStreamRef out,
	      aStreamRef err, 
	      aErr* pErr);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _aLeaf_H_ */
