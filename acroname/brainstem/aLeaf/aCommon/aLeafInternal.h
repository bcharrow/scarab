/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aLeafInternal.h                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent TEA compiler  */
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

#ifndef _aLeafInternal_H_
#define _aLeafInternal_H_

#include "aErr.h"
#include "aIO.h"
#include "aCCMap.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * type definitions
 */

typedef struct aLeaf {
  aStreamRef		errStream;
  int			check;
  void*			vpLAST;
} aLeaf;

#define aLEAFCHECK	0xED1E

#define aVALIDLEAF(p)	if ((!p) || 				   \
			(((aLeaf*)p)->check != aLEAFCHECK))	   \
			  leafErr = aErrParam;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * routine definitions
 */

aErr aLeafInternal_Create(aLeaf** ppLeaf);

aErr aLeafInternal_Compile(aLeaf* pLeaf,
			   aStreamRef source,
			   const char* sourceName,
			   int flags,
			   aStreamRef result,
			   aStreamRef out,
			   aStreamRef err);
aErr aLeafInternal_Destroy(aLeaf* pLeaf);

#endif /* _aLeafInternal_H_ */
