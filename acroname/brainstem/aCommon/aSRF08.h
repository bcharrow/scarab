/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSRF08.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of cross-platform SRF08 Ranger          */
/*              module interface routines.			   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/*        docs: Documentation can be found in the BrainStem        */
/*		reference under the major topic "C", category      */
/*              aSRF08                                             */
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

#ifndef _aSRF08_H_
#define _aSRF08_H_

#include "aIO.h"
#include "aStem.h"
#include "aSRF08Defs.tea"


#ifdef __cplusplus
extern "C" {
#endif

aErr aSRF08_GetRange(aStemLib stemLib,
		     const unsigned char router,
		     const unsigned char srf08address,
		     const unsigned char units,
		     short* pVals,
		     const unsigned char nVals);

aErr aSRF08_GetLight(aStemLib stemLib,
		     const unsigned char router,
		     const unsigned char srf08address,
		     unsigned char* pVal);

#ifdef __cplusplus 
}
#endif

#endif /* _aSRF08_H_ */
