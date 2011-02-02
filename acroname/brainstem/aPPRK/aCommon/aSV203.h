/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSV203.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent and 	   */
/*		SV203 PPRK communication layer.			   */
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

#ifndef _aSV203_H_
#define _aSV203_H_

#include "aPPRKInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * SV203 constants
 */

/* maximum delay before timeout when awaiting replies */
#define aSV203DELAYMS	200

/* minimum delay between commands being sent to the SV203 */
#define aSV203CMDSPACINGMS	20



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * OS Independant Routines
 */

aErr aSV203_Init(aPPRK* pPPRK);

aErr aSV203_EnsureSpacing(aPPRK* pPPRK);

aErr aSV203_SetServo(aPPRK* pPPRK,
		     unsigned int nServoNum,
		     unsigned int nAbsolutePos);

aErr aSV203_GetRange(aPPRK* pPPRK,
		     unsigned int nRangeNum,
		     float* pValue);

#endif /* _aSV203_H_ */
