/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSteepGenInternal.h					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of backend code generator for the	   */
/*		the TEA virtual machine. 	                   */
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

#ifndef _aSteepGenInternal_H_
#define _aSteepGenInternal_H_

#include "aSAST.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * type definitions
 */

typedef struct aSteepGen {
  int			check;
} aSteepGen;

#define aSTEEPGENCHECK	0xFEED

#define aVALIDSTEEPGEN(p)	if ((!p) || 				   \
			(((aSteepGen*)p)->check != aSTEEPGENCHECK))	   \
			  sgErr = aErrParam;

aErr aSteepGenInternal_Create(aSteepGen** ppSteepGen);

aErr aSteepGenInternal_Generate(aSteepGen* pSteepGen,
				aSAST* pSAST,
			        aStreamRef result);

aErr aSteepGenInternal_Destroy(aSteepGen* pSteepGen);


#endif /* _aSteepGenInternal_H_ */
