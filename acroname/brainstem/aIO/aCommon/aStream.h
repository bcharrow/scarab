
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStream.h                                                 */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Cross-Platform definition of stream I/O		   */
/*              routines.                                          */
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

#ifndef _aStream_H_
#define _aStream_H_

typedef aErr (*aStreamWriteProc)(const char* pData,
				 const unsigned long nSize, 
			         void* ref);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream object definition
 */

typedef struct aStream {
  aIOLib		libRef; /* must be first element! */
  aStreamGetProc	getProc;
  aStreamPutProc	putProc;
  aStreamWriteProc	writeProc;
  aStreamDeleteProc	deleteProc;
  void*			procRef;

  /* pieces for handling buffered reading */ 
  char*			pBuffer;
  unsigned int		bufCount;
  unsigned int		bufSize;
  unsigned int		bufFirst;
  unsigned int		bufLast;

  /* check for stream integrity */
  int			check;

} aStream;

#define aSTREAMCHECK 0x4321

#define aVALIDSTREAM(sr)						                               \
  if ((sr == NULL) ||                                              \
      (((aStream*)sr)->check != aSTREAMCHECK)) {	   	   \
    err = aErrParam;						   \
  }

#define aSTREAMBUFCHUNK		16

#endif /* _aStream_H_ */
