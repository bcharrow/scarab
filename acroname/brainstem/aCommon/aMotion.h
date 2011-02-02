/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aMotion.h                                                 */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Routine definitions for accessing the motion       */
/*              settings on the BrainStem.			   */
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

#ifndef _aMotion_H_
#define _aMotion_H_

#include "aMotionDefs.tea"
#include "aStem.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Motion routines
 */

#ifdef __cplusplus
extern "C" {
#endif

aErr aMotion_GetMode (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  unsigned char* pVal,
  unsigned char* pFlags
);

aErr aMotion_SetMode (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const unsigned char val,
  const unsigned char flags
);

aErr aMotion_GetParam (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const unsigned char paramIndex,
  short* pVal
);

aErr aMotion_SetParam (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const unsigned char paramIndex,
  const short val
);

aErr aMotion_GetRampParam (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const unsigned char paramIndex,
  short* pVal
);

aErr aMotion_SetRampParam (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const unsigned char paramIndex,
  const short val
);

aErr aMotion_SaveParams (
  aStemLib stemRef,
  const unsigned char module,
  const int nChannel
);

aErr aMotion_SetValue (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  const short position
);

aErr aMotion_GetValue (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  short* pPosition
);

aErr aMotion_GetEnc32 (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  char* pBuffer
);

aErr aMotion_GetPIDInput (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char channel,
  short* pValue
);

aErr aMotion_DecodeFrequency (
  const short paramVal,
  long* pFrequency
);

aErr aMotion_RampEnable (
  aStemLib stemRef,
  const unsigned char module,
  const unsigned char flags,
  const unsigned char channel
);

#ifdef __cplusplus 
}
#endif

#endif /* aMotion_H_ */

