/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aDigital.h                                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: BrainStem digital IO routines.                     */
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

#ifndef _aDigital_H_
#define _aDigital_H_

#include "aStem.h"
#include "aStemCore.h"


/* defines for setting digital configuration */
/* these may be ORed together to configure a digital IO pin */

#define	aDIGITAL_OUTPUT   0
#define	aDIGITAL_INPUT    1
#define	aDIGITAL_POLLENA  2
#define	aDIGITAL_POLLHI   4
#define	aDIGITAL_TMRHI    8
#define	aDIGITAL_TMRWID   16


#ifdef __cplusplus
extern "C" {
#endif

aErr aDigital_ReadInt(const aStemLib stemRef,
		      const unsigned char module,
		      const unsigned char nDigitalIndex,
		      int* pDigitalValue);

aErr aDigital_ReadTmr(const aStemLib stemRef,
		      const unsigned char module,
		      const unsigned char nDigitalIndex,
		      int* pTimerValue);

aErr aDigital_GetConfig(const aStemLib stemRef,
			const unsigned char module,
			const unsigned char nDigitalIndex,
			int* pConfigValue);

aErr aDigital_WriteInt(const aStemLib stemRef,
		       const unsigned char module,
		       const unsigned char nDigitalIndex,
		       int nDigitalValue);

aErr aDigital_SetConfig(const aStemLib stemRef,
			const unsigned char module,
			const unsigned char nDigitalIndex,
			int nConfigValue);

#ifdef __cplusplus
}
#endif

#endif /* _aDigital_H_ */
