/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aPPRKInternal.h					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent and 	   */
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

#ifndef _aPPRKInternal_H_
#define _aPPRKInternal_H_

#include "aUI.h"
#include "aPPRK.h"
#include "aStem.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * supported controller types
 */

typedef enum {
  kCtlUnknown = 0,
  kCtlBrainStem,
  kCtlSV203
} aPPRKCtlType;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * definition of the PPRK object
 */

typedef struct aSV203Cntl {
  char			cmd[20];
  int			cmdLen;
  int			nesting;
  unsigned long		msLastCmd;
} aSV203Cntl;

typedef struct aBrainStemCntl {
  aStemLib		stemRef;
  unsigned char		address;
  aBool			bInited;
} aBrainStemCntl;

typedef struct aPPRK {
  aIOLib		ioRef;
  aUILib		uiRef;
  aStreamRef		linkStream;
  aSettingFileRef	settingFile;
  aPPRKCtlType		eCtlType;
  unsigned int		lastServo;
  float			servo_neutral[3];
  float			servo_current[3];
  unsigned int		servo_index[3];
  unsigned int		ranger_index[3];
  pprkSerialFlushProc	flushProc;
  void*			flushProcRef;

  union {
    aSV203Cntl		sv203;
    aBrainStemCntl	brainstem;
  } controller;

  int			check;
} aPPRK;

#define aPPRKCHECK	0xBEEF

#define aVALIDPPRK(p)	if ((!p) || 				   \
			(((aPPRK*)p)->check != aPPRKCHECK))	   \
			  pprkErr = aErrParam;

#define aPPRKMAXSETTINGLEN	32
#define aPPRKSETTINGFILE	"aPPRK.config"

#ifdef aPALM
#define aPPRKSERPORTDEFAULT	"serial"
#endif /* aPALM */

#ifdef aWIN
#define aPPRKSERPORTDEFAULT	"COM1"
#endif /* aWIN */

#ifdef aUNIX
#ifdef aMACX
#define aPPRKSERPORTDEFAULT	"tty.KeyUSA19QI121.1"
#else /* aMACX */
#define aPPRKSERPORTDEFAULT	"ttyS0"
#endif /* aMACX */
#endif /* aUNIX */

#ifdef aWINCE
#define aPPRKSERPORTDEFAULT	"COM1:"
#endif /* aWINCE */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * routine definitions
 */

aErr aPPRKInternal_Create(aPPRK** ppPPRK);

#if 0
aErr aPPRKInternal_SetFlush(aPPRK* pPPRK,
			    pprkSerialFlushProc flushProc,
			    void* procRef);
#endif

aErr aPPRKInternal_TimeSlice(aPPRK* pPPRK);

aErr aPPRKInternal_SetServoAbs(aPPRK* pPPRK,
			       const unsigned int nServoNum,
			       float setting);

aErr aPPRKInternal_SetServoRel(aPPRK* pPPRK,
			       const unsigned int nServoNum,
			       float offset);

aErr aPPRKInternal_GetServo(aPPRK* pPPRK,
			    const unsigned int nServoNum,
			    float* pSetting);

aErr aPPRKInternal_GetRange(aPPRK* pPPRK,
			    const unsigned int nRangerNum,
			    float* pValue);

aErr aPPRKInternal_Sleep(aPPRK* pPPRK,
			 const unsigned long nMilliSec);

aErr aPPRKInternal_Destroy(aPPRK* pPPRK);

#endif /* _aPPRKInternal_H_ */
