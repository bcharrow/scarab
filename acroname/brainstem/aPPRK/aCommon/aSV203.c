/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSV203.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent Pontech   */
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

#include "aSV203.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * end of record character used for commands to the SV203
 */

#define eolChar 13


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSV203_Init
 */

aErr aSV203_Init(aPPRK* pPPRK)
{
  aErr pprkErr = aErrNone;
  char command[] = {'B', 'D', '1', eolChar};

  aAssert(pPPRK);

  /* bogus servo number to set proper servo */
  pPPRK->lastServo = 100;

  /* initialize to board 1 (default) */
  aStream_Write(pPPRK->ioRef,
  		pPPRK->linkStream,
  		command, 
  		4,
  		&pprkErr);

  if ((pprkErr == aErrNone)
      && (pPPRK->flushProc))
    pprkErr = pPPRK->flushProc(pPPRK->flushProcRef);
  
  return pprkErr;

} /* aSV203_Init */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSV203_EnsureSpacing
 *
 * Make sure enough time has passed since the previous command was
 * sent.
 */

aErr aSV203_EnsureSpacing(aPPRK* pPPRK)
{
  aErr pprkErr = aErrNone;
  unsigned long now = 0;
  unsigned long ready = pPPRK->controller.sv203.msLastCmd 
  			+ aSV203CMDSPACINGMS;

  aIO_GetMSTicks(pPPRK->ioRef, &now, &pprkErr);

  if ((pprkErr == aErrNone)
      && (ready > now))
    pprkErr = aPPRKInternal_Sleep(pPPRK, ready - now);

  pPPRK->controller.sv203.msLastCmd = now;

  return pprkErr;

} /* aSV203_EnsureSpacing */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSV203_SetServo
 */

aErr aSV203_SetServo(aPPRK* pPPRK,
		     unsigned int nServoNum,
		     unsigned int nAbsolutePos)
{
  aErr pprkErr = aErrNone;
  char num[10];
  char command[30];

  aAssert(pPPRK);
  aAssert(nServoNum < 3);
  aAssert(nAbsolutePos < 256);

  aAssert(pPPRK->controller.sv203.nesting >= 0);
  pPPRK->controller.sv203.nesting++;

  aStringCopy(command, "");

  /* only post the servo number if it has changed */
  if (nServoNum != pPPRK->lastServo) {
    aStringCat(command, "SV");
    aStringFromInt(num, (1 + nServoNum));
    aStringCat(command, num);
    aStringCat(command, ",");
    pPPRK->lastServo = nServoNum;
  }

  aStringCat(command, "M");
  aStringFromInt(num, nAbsolutePos);
  aStringCat(command, num);

  /* add a comma after preceeding commands */
  if (pPPRK->controller.sv203.cmdLen > 0) {
    pPPRK->controller.sv203.cmd[pPPRK->controller.sv203.cmdLen++] = ',';
    pPPRK->controller.sv203.cmd[pPPRK->controller.sv203.cmdLen] = 0;
  }

  /* add on the command */
  aStringCat(pPPRK->controller.sv203.cmd, command);
  pPPRK->controller.sv203.cmdLen += (int)aStringLen(command);

  if (pPPRK->controller.sv203.nesting == 1) {
    pPPRK->controller.sv203.cmd[pPPRK->controller.sv203.cmdLen++] = eolChar;

    /* ensure spacing between this and previous command */
    if (pprkErr == aErrNone)
      pprkErr = aSV203_EnsureSpacing(pPPRK);
    
    if (pprkErr == aErrNone) {
      aStream_Write(pPPRK->ioRef,
  		    pPPRK->linkStream,
  		    pPPRK->controller.sv203.cmd, 
  		    (unsigned long)pPPRK->controller.sv203.cmdLen,
  		    &pprkErr);
      pPPRK->controller.sv203.cmdLen = 0;
      pPPRK->controller.sv203.cmd[0] = 0;
    }
  }

  if ((pprkErr == aErrNone)
      && (pPPRK->flushProc))
    pprkErr = pPPRK->flushProc(pPPRK->flushProcRef);

  pPPRK->controller.sv203.nesting--;
  aAssert(pPPRK->controller.sv203.nesting >= 0);

  return pprkErr;

} /* aSV203_SetServo */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSV203_GetRange
 */

aErr aSV203_GetRange(aPPRK* pPPRK,
		     unsigned int nRangeNum,
		     float* pValue)
{
  aErr pprkErr = aErrNone;
  char num[10];
  char command[30];
  int rawValue;
  unsigned char val;
  unsigned long startTime;
  unsigned long now;

  aAssert(pPPRK);
  aAssert(nRangeNum < 3);
  aAssert(pValue);

  aAssert(pPPRK->controller.sv203.nesting >= 0);
  pPPRK->controller.sv203.nesting++;

  aStringCopy(command, "AD");
  aStringFromInt(num, (1 + nRangeNum));
  aStringCat(command, num);

  /* add a comma after preceeding commands */
  if (pPPRK->controller.sv203.cmdLen > 0) {
    pPPRK->controller.sv203.cmd[pPPRK->controller.sv203.cmdLen++] = ',';
    pPPRK->controller.sv203.cmd[pPPRK->controller.sv203.cmdLen] = 0;
  }

  /* add on the command */
  aStringCat(pPPRK->controller.sv203.cmd, command);
  pPPRK->controller.sv203.cmdLen += (int)aStringLen(command);

  /* add the record terminator */
  pPPRK->controller.sv203.cmd[pPPRK->controller.sv203.cmdLen++] = eolChar;

  /* ensure spacing between this and previous command */
  if (pprkErr == aErrNone)
    pprkErr = aSV203_EnsureSpacing(pPPRK);

  if (pprkErr == aErrNone) {
    aStream_Write(pPPRK->ioRef,
  		  pPPRK->linkStream,
  		  pPPRK->controller.sv203.cmd, 
  		  (unsigned long)pPPRK->controller.sv203.cmdLen,
  		  &pprkErr);
    pPPRK->controller.sv203.cmdLen = 0;
    pPPRK->controller.sv203.cmd[0] = 0;
  }

  /* establish timing for timeout */
  if (pprkErr == aErrNone)
    aIO_GetMSTicks(pPPRK->ioRef, &startTime, &pprkErr); 

  if (pprkErr == aErrNone) {
    rawValue = 0;
    do {
      aStream_Read(pPPRK->ioRef, pPPRK->linkStream,
    	         (char*)&val, 1, &pprkErr);
      if (pprkErr == aErrNone) {
        if (val == eolChar) {
          break;
        } else {
          if ((val >= '0') && (val <= '9'))
            rawValue = rawValue * 10 + (val - '0');
        }
      }

      else if (pprkErr == aErrNotReady)
        pprkErr = aErrNone;

      if (pprkErr == aErrNone)
        aIO_GetMSTicks(pPPRK->ioRef, &now, &pprkErr); 

    } while (((startTime + aSV203DELAYMS) > now) 
             && (pprkErr == aErrNone));

    if ((startTime + aSV203DELAYMS) <= now)
      pprkErr = aErrTimeout;
    else
      *pValue = (float)((255 - rawValue) / 255.0f);
  }

  aAssert(pPPRK->controller.sv203.nesting >= 0);
  pPPRK->controller.sv203.nesting--;

  return pprkErr;

} /* aSV203_GetRange */
