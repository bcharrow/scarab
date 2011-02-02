/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStreamBuffer.h                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: cross-platform data buffer stream definition.      */
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

#include "aIO.h"

#ifndef _aStreamBuffer_H_
#define _aStreamBuffer_H_

#include "aStreamBuffer.h"

/* Here are some facts about the buffer:
 * + if Start and End are the same, the buffer is empty
 * + if End is one less that Start, the buffer is full
 */
typedef struct aStreamBufferData {
  aIOLib	ioRef;
  aMemSize	nStart; /* index of the first buffer character */
  aMemSize	nBytes; /* number of bytes currently in buffer */
  aMemSize	nIncSize;
  aMemSize	nBufferSize;
  char* 	pBuffer;
  int		check;
} aStreamBufferData;

#define aSBDCHECK	0xCDEF

#define aVALIDSBD(p)						   \
  if ((p == NULL) ||						   \
      (((aStreamBufferData*)p)->check != aSBDCHECK)) {  	   \
    err = aErrParam;						   \
  }

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static aErr sStreamBufferGet(char* pData, void* ref);
static aErr sStreamBufferPut(char* pData, void* ref);
static aErr sStreamBufferWrite(const char* pData, 
			       const unsigned long nSize,
			       void* ref);
static aErr sStreamBufferDelete(void* ref);
static aErr sEnsureContinuous(aStreamBufferData* pSBD);

#endif /* _aStreamBuffer_H_ */
