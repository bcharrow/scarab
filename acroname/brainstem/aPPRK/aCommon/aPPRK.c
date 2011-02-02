/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aPPRK.c                                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent and 	   */
/*		controller-independant PPRK communication layer.   */
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

#include "aPPRKInternal.h"
#include "aVersion.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRK_GetVersion
 */

aLIBRETURN aPPRK_GetVersion(aPPRKLib libRef,
			    unsigned long *pNVersion,
			    aErr* pErr)
{
  aErr pprkErr = aErrNone;

  if ((libRef == NULL) || (pNVersion == NULL))
    pprkErr = aErrParam;

  if (pprkErr == aErrNone)
    *pNVersion = (aVERSION_MAJOR << 24) 
    		+ (aVERSION_MINOR << 16)
    		+ aPPRK_BUILD_NUM;

  if (pErr != NULL)
    *pErr = pprkErr;

  return (aLIBRETURN)(pprkErr != aErrNone);

} /* aPPRK_GetVersion */


#ifdef aNORMALSLIB

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRK_GetLibRef
 */

aLIBRETURN aPPRK_GetLibRef(aPPRKLib* ppprkRef, 
			   aErr* pErr)
{
  aErr pprkErr = aPPRKInternal_Create((aPPRK**)ppprkRef);

  if (pErr != NULL)
    *pErr = pprkErr;

  return (aLIBRETURN)(pprkErr != aErrNone);

} /* aPPRK_GetLibRef */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRK_ReleaseLibRef
 */

aLIBRETURN aPPRK_ReleaseLibRef(aPPRKLib pprkRef, 
			       aErr* pErr)
{
  aErr pprkErr = aPPRKInternal_Destroy((aPPRK*)pprkRef);

  if (pErr != NULL)
    *pErr = pprkErr;

  return (aLIBRETURN)(pprkErr != aErrNone);

} /* aPPRKReleaseLibRef */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRK_TimeSlice
 */

aLIBRETURN aPPRK_TimeSlice(aPPRKLib pprkRef, 
			   aErr* pErr)
{
  aErr pprkErr = aPPRKInternal_TimeSlice((aPPRK*)pprkRef);

  if (pErr != NULL)
    *pErr = pprkErr;

  return (aLIBRETURN)(pprkErr != aErrNone);

} /* aPPRK_TimeSlice */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRK_SetServoAbs
 */

aLIBRETURN aPPRK_SetServoAbs(aPPRKLib pprkRef, 
			     const unsigned int nServoNum,
			     float setting,
			     aErr* pErr)
{
  aErr pprkErr = aPPRKInternal_SetServoAbs((aPPRK*)pprkRef, 
  					   nServoNum, setting);

  if (pErr != NULL)
    *pErr = pprkErr;

  return (aLIBRETURN)(pprkErr != aErrNone);

} /* aPPRK_SetServoAbs */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRK_SetServoRel
 */

aLIBRETURN aPPRK_SetServoRel(aPPRKLib pprkRef, 
			     const unsigned int nServoNum,
			     float offset,
			     aErr* pErr)
{
  aErr pprkErr = aPPRKInternal_SetServoRel((aPPRK*)pprkRef, 
  					   nServoNum, offset);

  if (pErr != NULL)
    *pErr = pprkErr;

  return (aLIBRETURN)(pprkErr != aErrNone);

} /* aPPRK_SetServoRel */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRK_GetServo
 */

aLIBRETURN aPPRK_GetServo(aPPRKLib pprkRef, 
			  const unsigned int nServoNum,
			  float* pSetting,
			  aErr* pErr)
{
  aErr pprkErr = aPPRKInternal_GetServo((aPPRK*)pprkRef, 
  					nServoNum, pSetting);

  if (pErr != NULL)
    *pErr = pprkErr;

  return (aLIBRETURN)(pprkErr != aErrNone);

} /* aPPRK_GetServo */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRK_GetRange
 */

aLIBRETURN aPPRK_GetRange(aPPRKLib pprkRef, 
			  const unsigned int nRangeNum,
			  float* pValue,
			  aErr* pErr)
{
  aErr pprkErr = aPPRKInternal_GetRange((aPPRK*)pprkRef, 
  					nRangeNum, pValue);

  if (pErr != NULL)
    *pErr = pprkErr;

  return (aLIBRETURN)(pprkErr != aErrNone);

} /* aPPRK_GetRange */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRK_Sleep
 */

aLIBRETURN aPPRK_Sleep(aPPRKLib pprkRef, 
		       const unsigned long nMilliSec,
		       aErr* pErr)
{
  aErr pprkErr = aPPRKInternal_Sleep((aPPRK*)pprkRef, nMilliSec);

  if (pErr != NULL)
    *pErr = pprkErr;

  return (aLIBRETURN)(pprkErr != aErrNone);

} /* aPPRK_Sleep */


#endif /* aNORMALSLIB */
