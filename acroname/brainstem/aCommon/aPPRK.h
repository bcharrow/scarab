/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aPPRK.h                                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent and 	   */
/*		controller independant PPRK communication layer.   */
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

#ifndef _aPPRK_H_
#define _aPPRK_H_

#include "aStem.h"
#include "aPPRKOSExport.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * library version number
 */

#define aPPRK_VERSION		0x1000


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * opaque reference to the I/O library
 */

typedef aLIBREF aPPRKLib;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * PPRK idle callback
 */
 
typedef aErr (*pprkSerialFlushProc)(void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * PPRK manipulation routines
 */

#ifdef __cplusplus
extern "C" {
#endif

aPPRK_EXPORT aLIBRETURN aPPRK_GetLibRef(aPPRKLib* pPPRKRef, 
					aErr* pErr);

aPPRK_EXPORT aLIBRETURN aPPRK_ReleaseLibRef(aPPRKLib PPRKRef, 
					    aErr* pErr);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * PPRK Routine definitions
 */

aPPRK_EXPORT aLIBRETURN aPPRK_GetVersion(aPPRKLib PPRKRef,
					 unsigned long *pNVersion,
					 aErr* pErr)
		aPPRK_TRAP(aPPRKGetVersionTrap);

aPPRK_EXPORT aLIBRETURN aPPRK_TimeSlice(aPPRKLib PPRKRef,
					aErr* pErr)
		aPPRK_TRAP(aPPRKTimeSliceTrap);

aPPRK_EXPORT aLIBRETURN aPPRK_SetServoAbs(aPPRKLib PPRKRef,
				          const unsigned int nServoNum,
				          float setting,
				          aErr* pErr)
		aPPRK_TRAP(aPPRKSetServoAbsTrap);

aPPRK_EXPORT aLIBRETURN aPPRK_SetServoRel(aPPRKLib PPRKRef,
				          const unsigned int nServoNum,
				          float offset,
				          aErr* pErr)
		aPPRK_TRAP(aPPRKSetServoAbsTrap);

aPPRK_EXPORT aLIBRETURN aPPRK_GetServo(aPPRKLib PPRKRef,
				       const unsigned int nServoNum,
				       float* pSetting,
				       aErr* pErr)
		aPPRK_TRAP(aPPRKGetServoTrap);

aPPRK_EXPORT aLIBRETURN aPPRK_GetRange(aPPRKLib PPRKRef,
				       const unsigned int nRangerNum,
				       float* pValue,
				       aErr* pErr)
		aPPRK_TRAP(aPPRKGetRangeTrap);

aPPRK_EXPORT aLIBRETURN aPPRK_Sleep(aPPRKLib PPRKRef,
				    const unsigned long nMilliSec,
				    aErr* pErr)
		aPPRK_TRAP(aPPRKSleepTrap);

#ifdef __cplusplus
}
#endif

#endif /* _aPPRK_H_ */
