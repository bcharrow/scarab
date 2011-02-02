/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSteep.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: export definitions for the aSteep object.          */
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

#ifndef _aSteep_H_
#define _aSteep_H_

#include "aIO.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * opaque reference to the compiler library
 */

typedef aLIBREF aSteepLib;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * define symbol import mechanism
 */

#ifndef aSTEEP_EXPORT
#define aSTEEP_EXPORT aLIB_IMPORT
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * compiler flags
 */

#define	fSteepPreprocess		0x01
#define fSteepSAST	 		0x02
#define fSteepGenerateCode 		0x04
#define fSteepDisassemble		0x08
#define fSteepGenerateSymbols		0x10


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * compiler library manipulation routines
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

aSTEEP_EXPORT aLIBRETURN 
aSteep_GetLibRef(aSteepLib* pSteepRef, 
		 aErr* pErr);

aSTEEP_EXPORT aLIBRETURN 
aSteep_ReleaseLibRef(aSteepLib steepRef, 
		     aErr* pErr);

aSTEEP_EXPORT aLIBRETURN 
aSteep_GetVersion(aSteepLib steepRef,
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

aSTEEP_EXPORT aLIBRETURN 
aSteep_Compile(aSteepLib steepRef,
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

#endif /* _aSteep_H_ */
