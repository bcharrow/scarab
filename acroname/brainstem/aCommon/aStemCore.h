/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStemCore.h     	                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of core access routines for the 	   */
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

#ifndef _aStemCore_H_
#define _aStemCore_H_

#include "aStem.h"
#include "aTEA.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


aBool aStemCore_CmdFilter(const unsigned char module,
			  const unsigned char dataLength,
			  const char* data,
			  void* ref);


#if 0
aErr aStemCore_LaunchVM(aStemLib stemLib,
			const unsigned char module,
			const unsigned char fileSlot,
			const char* pProgramData,
			const unsigned char nProgramDataLen,
        		const unsigned char flags,
			aTEAProcessID* pPID);

aLIBRETURN aStem_GetA2D_Int(aStemLib stemLib,
			    const unsigned char module,
			    const unsigned char nAnalogIndex,
			    int *pAnalogValue,
			    aErr* pErr);

aLIBRETURN aStem_GetServoPosition(aStemLib stemLib,
				  const unsigned char module,
				  const unsigned char nServoIndex,
				  unsigned char *pServoPosition,
				  aErr* pErr);

aLIBRETURN aStem_GetServoConfig(aStemLib stemLib,
				const unsigned char module,
				const unsigned char nServoIndex,
				unsigned char *pConfigByte,
				aErr* pErr);

aLIBRETURN aStem_GetServoLimits(aStemLib stemLib,
				const unsigned char module,
				const unsigned char nServoIndex,
				unsigned char *pLimitBytes,
				aErr* pErr);

aLIBRETURN aStem_SetServoPositionAbs(aStemLib stemLib,
				     const unsigned char module,
				     const unsigned char nServoIndex,
				     const unsigned char nPosition,
				     aErr* pErr);

aLIBRETURN aStem_SetServoPositionRel(aStemLib stemLib,
				     const unsigned char module,
				     const unsigned char nServoIndex,
				     const unsigned char nChange,
				     aErr* pErr);

aLIBRETURN aStem_SetServoConfig(aStemLib stemLib,
				const unsigned char module,
				const unsigned char nServoIndex,
				const unsigned char configByte,
				aErr* pErr);

aLIBRETURN aStem_SetServoLimits(aStemLib stemLib,
				const unsigned char module,
				const unsigned char nServoIndex,
				const unsigned char *pLimitBytes,
				aErr* pErr);

aLIBRETURN aServo_CommitSettings(aStemLib stemLib,
				 const unsigned char module,
				 aErr* pErr);
#endif

#ifdef __cplusplus 
}
#endif

#endif /* _aStemCore_H_ */
