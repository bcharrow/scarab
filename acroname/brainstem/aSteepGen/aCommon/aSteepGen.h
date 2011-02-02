/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSteepGen.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of TEAvm code generator back end for    */
/*		the TEA language.	 	                   */
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

#ifndef _aSteepGen_H_
#define _aSteepGen_H_

#include "aSAST.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * opaque reference to the Optimization library
 */

typedef aLIBREF aSteepGenLib;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * define symbol import mechanism
 */

#ifndef aSTEEPGEN_EXPORT
#define aSTEEPGEN_EXPORT aLIB_IMPORT
#endif


#ifdef __cplusplus
extern "C" {
#endif

aSTEEPGEN_EXPORT aLIBRETURN 
aSteepGen_GetLibRef(aSteepGenLib* pSteepGenRef, 
		    aErr* pErr);
  
aSTEEPGEN_EXPORT aLIBRETURN 
aSteepGen_ReleaseLibRef(aSteepGenLib steepGenRef, 
			aErr* pErr);
  
aSTEEPGEN_EXPORT aLIBRETURN 
aSteepGen_Generate(aSteepGenLib steepGenRef,
		   aSAST* pSAST,
		   aStreamRef result, 
		   aErr* pErr);

#ifdef __cplusplus 
}
#endif

#endif /* _aSteepGen_H_ */
