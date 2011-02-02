/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aPPRKInternal.c					   */
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

#include "aPPRK.h"
#include "aPPRKInternal.h"
#include "aStreamUtil.h"
#include "aBrainStem.h"
#include "aSV203.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sPPRKInternal_SetupSerial(aPPRK* pPPRK);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPPRKInternal_SetupSerial
 */

aErr sPPRKInternal_SetupSerial(aPPRK* pPPRK)
{
  aErr pprkErr = aErrNone;
  
  aStreamUtil_CreateRawSettingStream(pPPRK->ioRef,
  				     pPPRK->uiRef,
  				     pPPRK->settingFile,
  				     &pPPRK->linkStream,
  				     &pprkErr);

  return pprkErr;

} /* sPPRKInternal_SetupSerial */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRKInternal_Create
 */

aErr aPPRKInternal_Create(aPPRK** ppPPRK)
{
  aErr pprkErr = aErrNone;
  aErr settingErr = aErrNone;
  aPPRK* pPPRK = NULL;

  if (ppPPRK == NULL)
    pprkErr = aErrParam;

  if (pprkErr == aErrNone) {
    pPPRK = (aPPRK*)aMemAlloc(sizeof(aPPRK));
    if (pPPRK == NULL) {
      pprkErr = aErrParam;
    } else {
      aBZero(pPPRK, sizeof(aPPRK));
      pPPRK->check = aPPRKCHECK;
    }
    if (pprkErr == aErrNone)
      *ppPPRK = pPPRK;
    else
      aPPRKInternal_Destroy(pPPRK);
  }

  /* get the io library reference */
  if (pprkErr == aErrNone)
    aIO_GetLibRef(&pPPRK->ioRef, &pprkErr);

  /* get the ui library reference */
  if (pprkErr == aErrNone)
    aUI_GetLibRef(&pPPRK->uiRef, &pprkErr);

  /* try to establish settings */
  if (pprkErr == aErrNone)
    aSettingFile_Create(pPPRK->ioRef,
    			aPPRKMAXSETTINGLEN,
    			aPPRKSETTINGFILE,
    			&pPPRK->settingFile,
    			&pprkErr);

  /* try to establish serial link to controller */
  if (pprkErr == aErrNone)
    pprkErr = sPPRKInternal_SetupSerial(pPPRK);

  /* find out what type of controller we are using */
  if (pprkErr == aErrNone) {
    char* controller;
    
    aSettingFile_GetString(pPPRK->ioRef, pPPRK->settingFile,
    			   "controller", &controller,
    			   "BrainStem",
    			   &pprkErr);

    if (pprkErr == aErrNone) {
      if (!aStringCompare(controller, "BrainStem")) {
        pPPRK->eCtlType = kCtlBrainStem;
        pprkErr = aBrainStem_Init(pPPRK);
      } else if (!aStringCompare(controller, "SV203")) {
        pPPRK->eCtlType = kCtlSV203;
        pprkErr = aSV203_Init(pPPRK);
      }
    }
  }

  /* get the settings */
  if (pprkErr == aErrNone) {
    int index;
  
    /* neutral positions */
    aSettingFile_GetFloat(pPPRK->ioRef, pPPRK->settingFile,
    			  "servo_neutral_0", 
    			  &pPPRK->servo_neutral[0],
    			  0.0f,
    			  NULL);
    aSettingFile_GetFloat(pPPRK->ioRef, pPPRK->settingFile,
    			  "servo_neutral_1", 
    			  &pPPRK->servo_neutral[1],
    			  0.0f,
    			  NULL);
    aSettingFile_GetFloat(pPPRK->ioRef, pPPRK->settingFile,
    			  "servo_neutral_2", 
    			  &pPPRK->servo_neutral[2],
    			  0.0f,
    			  NULL);

    /* servo index mapping */
    aSettingFile_GetInt(pPPRK->ioRef, pPPRK->settingFile,
    			"servo_index_0", 
    			&index, 0, &settingErr);

    if (settingErr == aErrNone) {
      if (index > 2) index = 2;
      if (index < 0) index = 0;
      pPPRK->servo_index[0] = (unsigned int)index;
    }

    aSettingFile_GetInt(pPPRK->ioRef, pPPRK->settingFile,
    			"servo_index_1", 
    			&index, 1, &settingErr);

    if (settingErr == aErrNone) {
      if (index > 2) index = 2;
      if (index < 0) index = 0;
      pPPRK->servo_index[1] = (unsigned int)index;
    }

    aSettingFile_GetInt(pPPRK->ioRef, pPPRK->settingFile,
    			"servo_index_2", 
    			&index, 2, &settingErr);

    if (settingErr == aErrNone) {
      if (index > 2) index = 2;
      if (index < 0) index = 0;
      pPPRK->servo_index[2] = (unsigned int)index;
    }

    /* ranger index mapping */
    aSettingFile_GetInt(pPPRK->ioRef, pPPRK->settingFile,
    			"ranger_index_0", 
    			&index, 0, &settingErr);
    if (settingErr == aErrNone) {
      if (index > 2) index = 2;
      if (index < 0) index = 0;
      pPPRK->ranger_index[0] = (unsigned int)index;
    }

    aSettingFile_GetInt(pPPRK->ioRef, pPPRK->settingFile,
    			"ranger_index_1", 
    			&index, 1,
    			&settingErr);

    if (settingErr == aErrNone) {
      if (index > 2) index = 2;
      if (index < 0) index = 0;
      pPPRK->ranger_index[1] = (unsigned int)index;
    }

    aSettingFile_GetInt(pPPRK->ioRef, pPPRK->settingFile,
    			"ranger_index_2", 
    			&index, 2, &settingErr);
    if (settingErr == aErrNone) {
      if (index > 2) index = 2;
      if (index < 0) index = 0;
      pPPRK->ranger_index[2] = (unsigned int)index;
    }
  }

  /* set the servos to neutral to start */
  if (pprkErr == aErrNone)
    pprkErr = aPPRKInternal_SetServoAbs(pPPRK, 0, 0.0f);
  if (pprkErr == aErrNone)
    pprkErr = aPPRKInternal_SetServoAbs(pPPRK, 1, 0.0f);
  if (pprkErr == aErrNone)
    pprkErr = aPPRKInternal_SetServoAbs(pPPRK, 2, 0.0f);

  return pprkErr;

} /* aPPRKInternal_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRKInternal_TimeSlice
 */

aErr aPPRKInternal_TimeSlice(aPPRK* pPPRK)
{
  aErr pprkErr = aErrNone;

  aVALIDPPRK(pPPRK);

  if (pprkErr == aErrNone) {
    switch (pPPRK->eCtlType) {

    case kCtlBrainStem:
      pprkErr = aBrainStem_TimeSlice(pPPRK);
      break;

    /* the others do nothing here */  
    default:
      break;

    } /* controller type switch */
  }
  
  return pprkErr;

} /* aPPRKInternal_TimeSlice */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRKInternal_SetServoAbs
 */

aErr aPPRKInternal_SetServoAbs(aPPRK* pPPRK,
			       const unsigned int nServoNum,
			       float setting)
{
  aErr pprkErr = aErrNone;
  int value;
  float rawSetting;

  aVALIDPPRK(pPPRK);

  if (nServoNum >= 3)
    pprkErr = aErrRange;

  if (pprkErr == aErrNone) {

    /* compute the setting from neutral */ 
    rawSetting = pPPRK->servo_neutral[nServoNum] + setting;

    /* clamp to an allowable range */
    if (rawSetting > 1.0f)
      rawSetting = 1.0f;
    else if (rawSetting < -1.0f)
      rawSetting = -1.0f;
 
    /* convert to a byte range setting */
    value = (int)(rawSetting * 128) + 128;
    
    if (value == 256)
      value = 255;

    switch (pPPRK->eCtlType) {

    case kCtlBrainStem:
      pprkErr = aBrainStem_SetServo(pPPRK, 
      				pPPRK->servo_index[nServoNum], 
      				(unsigned int)value);
      break;

    case kCtlSV203:
      pprkErr = aSV203_SetServo(pPPRK, 
      				pPPRK->servo_index[nServoNum], 
      				(unsigned int)value);
      break;

    default:
      pprkErr = aErrConfiguration;
      break;
    } /* controller type switch */
  }
  
  if (pprkErr == aErrNone)
    pPPRK->servo_current[nServoNum] = rawSetting;

  return pprkErr;

} /* aPPRKInternal_SetServoAbs */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRKInternal_SetServoRel
 */

aErr aPPRKInternal_SetServoRel(aPPRK* pPPRK,
			       const unsigned int nServoNum,
			       float offset)
{
  aErr pprkErr = aErrNone;
  float rawSetting;

  aVALIDPPRK(pPPRK);

  if (nServoNum >= 3)
    pprkErr = aErrRange;

  if (pprkErr == aErrNone) {
    rawSetting = pPPRK->servo_current[nServoNum] + offset;
    pprkErr = aPPRKInternal_SetServoAbs(pPPRK, nServoNum, 
    					rawSetting);
  }

  return pprkErr;

} /* aPPRKInternal_SetServoRel */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRKInternal_GetServo
 */

aErr aPPRKInternal_GetServo(aPPRK* pPPRK,
			    const unsigned int nServoNum,
			    float* pSetting)
{
  aErr pprkErr = aErrNone;
  
  aVALIDPPRK(pPPRK);
  
  if (pSetting == NULL)
    pprkErr = aErrParam;

  if ((pprkErr == aErrNone) && (nServoNum >= 3))
    pprkErr = aErrRange;
    
  if (pprkErr == aErrNone)
    *pSetting = pPPRK->servo_current[nServoNum];

  return pprkErr;

} /* aPPRKInternal_GetServo */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRKInternal_GetRange
 */

aErr aPPRKInternal_GetRange(aPPRK* pPPRK,
			    const unsigned int nRangerNum,
			    float* pValue)
{
  aErr pprkErr = aErrNone;

  aVALIDPPRK(pPPRK);

  if (nRangerNum >= 3)
    pprkErr = aErrRange;
   
  if (pprkErr == aErrNone) {
    switch (pPPRK->eCtlType) {
    
    case kCtlBrainStem:
      pprkErr = aBrainStem_GetRange(pPPRK, 
      				    pPPRK->ranger_index[nRangerNum], 
      				    pValue);
      break;

    case kCtlSV203:
      pprkErr = aSV203_GetRange(pPPRK, 
      				pPPRK->ranger_index[nRangerNum], 
      				pValue);
      break;

    default:
      pprkErr = aErrConfiguration;
      break;
    } /* controller type switch */
  }

  return pprkErr;

} /* aPPRKInternal_GetRange */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRKInternal_Sleep
 */

aErr aPPRKInternal_Sleep(aPPRK* pPPRK,
			 const unsigned long nMilliSec)
{
  aErr pprkErr = aErrNone;

  unsigned long then;
  unsigned long now = 0;

  aIO_GetMSTicks(pPPRK->ioRef, &then, &pprkErr);

  while ((then + nMilliSec > now)
         && (pprkErr == aErrNone)) {
    pprkErr = aPPRKInternal_TimeSlice(pPPRK);
    if (pprkErr == aErrNone)
      aIO_GetMSTicks(pPPRK->ioRef, &now, &pprkErr);
  }

  return pprkErr;

} /* aPPRKInternal_Sleep */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aPPRKInternal_Destroy
 */

aErr aPPRKInternal_Destroy(aPPRK* pPPRK)
{
  aErr pprkErr = aErrNone;
  aErr ioErr;

  aVALIDPPRK(pPPRK);
  
  if (pprkErr == aErrNone) {
    switch (pPPRK->eCtlType) {

    case kCtlBrainStem:
      pprkErr = aBrainStem_Destroy(pPPRK);
      break;

    default:
      break;
    } /* controller type switch */
  }

  if ((pprkErr == aErrNone) && (pPPRK->linkStream != NULL))
    aStream_Destroy(pPPRK->ioRef, pPPRK->linkStream, &ioErr);

  if (pPPRK->settingFile != NULL)
    aSettingFile_Destroy(pPPRK->ioRef, pPPRK->settingFile, NULL);

  if (pPPRK->uiRef != NULL)
    aUI_ReleaseLibRef(pPPRK->uiRef, NULL);

  if (pPPRK->ioRef != NULL)
    aIO_ReleaseLibRef(pPPRK->ioRef, NULL);

  if (pprkErr == aErrNone)
    aMemFree((aMemPtr)pPPRK);

  return pprkErr;

} /* aPPRKInternal_Destroy */
