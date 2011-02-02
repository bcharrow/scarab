/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aBrainStem.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent and 	   */
/*		BrainStem PPRK communication layer.		   */
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


#ifndef _aBrainStem_H_
#define _aBrainStem_H_

#include "aPPRKInternal.h"

#define aPPRKSTEMTIMEOUTMS	1000

aErr aBrainStem_Init(aPPRK* pPPRK);

aErr aBrainStem_TimeSlice(aPPRK* pPPRK);

aErr aBrainStem_SetServo(aPPRK* pPPRK,
			 unsigned int nServoNum,
			 unsigned int nAbsolutePos);

aErr aBrainStem_GetRange(aPPRK* pPPRK,
			 unsigned int nRangeNum,
			 float* pValue);

aErr aBrainStem_Destroy(aPPRK* pPPRK);

#endif /* _aBrainStem_H_ */
