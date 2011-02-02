/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStemMsg.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Msg codes and pretty printing routine		   */
/*		implementation.					   */
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

#include "aOSDefs.h"
#include "aStemMsg.h"

const char* stemMsgText[nStemMsg] = {

  /* 0 */
  "no iic ack",
  /* 1 */
  "not iic WR addr",
  /* 2 */
  "iic queue overflow",
  /* 3 */
  "iic queue underflow",
  /* 4 */
  "power up reset",
  /* 5 */
  "no mem ack",
  /* 6 */
  "mem read too long",
  /* 7 */
  "mem address error",
  /* 8 */
  "invalid command",
  /* 9 */
  "cmd too long",
  /* 10 */
  "cmd index error",
  /* 11 */
  "cmd param error",
  /* 12 */
  "cmd queue overflow",
  /* 13 */
  "cmd queue underflow",
  /* 14 */
  "link queue overflow",
  /* 15 */
  "timer conflict",
  /* 16 */
  "file init error",
  /* 17 */
  "file mode error",
  /* 18 */
  "file close error",
  /* 19 */
  "file size error",
  /* 20 */
  "msg too long",
  /* 21 */
  "bad raw input",
  /* 22 */
  "iic buff ovflw",
  /* 23 */
  "iic cmd ovflw",
  /* 24 */
  "iic cmd unflw",
  /* 25 */
  "vm not free",
  /* 26 */
  "vm exit",
  /* 27 */
  "cmd too short",
  /* 28 */
  "serial error"  
};


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemMsg_Format message formatting routine
 */

void aStemMsg_Format(unsigned char code, 
		     unsigned char fromAddr,
		     char *pBuffer, 
		     unsigned int nMaxLen)
{
  int len = 0;
  if (!pBuffer)
    return;
  
  aStringFromInt(pBuffer, fromAddr);
  aStringCatSafe(pBuffer, nMaxLen - 1, ":");
  
  len = (int)aStringLen(pBuffer);
  if ((nMaxLen - len) < 1)
    return;

  if (code >= nStemMsg) {
    aStringCopySafe(&pBuffer[len], (nMaxLen - len), "unknown message");
  } else {
    aStringCopySafe(&pBuffer[len], (nMaxLen - len), stemMsgText[code]);
  }

} /* aStemMsg_Format */
