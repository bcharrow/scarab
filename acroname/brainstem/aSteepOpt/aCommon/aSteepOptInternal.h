/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSteepOptInternal.h					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of language purifier and optimizer for  */
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

#ifndef _aSteepOptInternal_H_
#define _aSteepOptInternal_H_

#include "aSAST.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * type definitions
 */

typedef struct aSteepOpt {
  int			check;
} aSteepOpt;

#define aSTEEPOPTCHECK	0xEDDE

#define aVALIDSTEEPOPT(p)	if ((!p) || 				   \
			(((aSteepOpt*)p)->check != aSTEEPOPTCHECK))	   \
			  soErr = aErrParam;

aErr aSteepOptInternal_Create(aSteepOpt** ppSteepOpt);

aErr aSteepOptInternal_Optimize(aSteepOpt* pSteepOpt,
				aSAST* pSAST);

aErr aSteepOptInternal_Destroy(aSteepOpt* pSteepOpt);

#endif /* _aSteepOptInternal_H_ */
