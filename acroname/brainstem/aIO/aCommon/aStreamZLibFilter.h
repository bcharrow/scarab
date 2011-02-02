
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStream_CreateZLibFilter.h                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent I/O layer.    */
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

#ifndef _aStreamZLibFilter_H_
#define _aStreamZLibFilter_H_

#include "aIO.h"
#include "aStream.h"

#include "zlib.h"

typedef struct aStreamZLibFilter {
  aIOLib	ioRef;
  z_stream 	z;
  int           z_err;   /* error code for last stream operation */
  int           z_eof;   /* set if end of input file */
  Byte*		buf;
  aStream*	pFilteredStream;
  aFileMode	eMode;
  int		check;
} aStreamZLibFilter;

#define aZLFCHECK	0xCDEF

#define aVALIDZLF(p)						   \
  if ((p == NULL) ||						   \
      (((aStreamZLibFilter*)p)->check != aZLFCHECK)) {  	   \
    err = aErrParam;						   \
  }

static aErr sZLF_Write (
  const char* pData, 
  const unsigned long nSize,
  void* ref
);
static aErr sZLF_Put (
  char* pData, 
  void* ref
);
static aErr sZLF_Get (
  char* pData, 
  void* ref
);
static aErr sZLF_Delete (
  void* ref
);


#endif /* _aStreamZLibFilter_H_ */
