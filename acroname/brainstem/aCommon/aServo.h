/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aServo.h                                                  */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of servo control routines for the   */
/*		BrainStem modules.	    			   */
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

#ifndef _aServo_H_
#define _aServo_H_

#include "aStem.h"
#include "aStemCore.h"


/* defines for setting servo configuration */
/* these may be ORed together to configure a servo output */

#define	aSERVO_DSTA		32
#define	aSERVO_INV		64
#define	aSERVO_ENA		128
#define	aSERVO_SPEEDMASK	0x0F


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

aErr aServo_GetPosition(aStemLib stemLib,
			const unsigned char module,
			const unsigned char nServoIndex,
			unsigned char *pServoPosition);

aErr aServo_GetConfig(aStemLib stemLib,
		      const unsigned char module,
		      const unsigned char nServoIndex,
		      unsigned char *pConfigByte);

aErr aServo_GetLimits(aStemLib stemLib,
		      const unsigned char module,
		      const unsigned char nServoIndex,
		      unsigned char *pLimitBytes);

aErr aServo_SetPositionAbs(aStemLib stemLib,
			   const unsigned char module,
			   const unsigned char nServoIndex,
			   const unsigned char nPosition);

aErr aServo_SetPositionRel(aStemLib stemLib,
			   const unsigned char module,
			   const unsigned char nServoIndex,
			   const int nChange);

aErr aServo_SetConfig(aStemLib stemLib,
		      const unsigned char module,
		      const unsigned char nServoIndex,
		      const unsigned char configByte);

aErr aServo_SetLimits(aStemLib stemLib,
		      const unsigned char module,
		      const unsigned char nServoIndex,
		      const unsigned char *pLimitBytes);

aErr aServo_CommitSettings(aStemLib stemLib,
			   const unsigned char module);

#ifdef __cplusplus 
}
#endif

#endif /* _aServo_H_ */
