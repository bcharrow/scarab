/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aErr.h                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Error code definitions for all routines.           */
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

#ifndef _aErr_H_
#define _aErr_H_

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Error codes for all routines.
 */

typedef enum aErr {

  aErrNone = 0,		/* 0 */
  aErrMemory,		/* 1 */
  aErrParam,		/* 2 */
  aErrNotFound,		/* 3 */
  aErrFileNameLength,	/* 4 */
  aErrBusy,		/* 5 */
  aErrIO,		/* 6 */
  aErrMode,		/* 7 */
  aErrWrite,		/* 8 */
  aErrRead,		/* 9 */
  aErrEOF,		/* 10 */
  aErrNotReady,		/* 11 */

  aErrPermission,	/* 12 */
  aErrRange,		/* 13 */
  aErrSize,		/* 14 */
  aErrOverrun,		/* 15 */

  aErrParse,		/* 16 */

  aErrConfiguration,	/* 17 */
  aErrTimeout,		/* 18 */
  
  aErrInitialization,	/* 19 */
  
  aErrVersion,		/* 20 */
  
  aErrUnimplemented,	/* 21 */

  aErrDuplicate,	/* 22 */

  aErrCancel,		/* 23 */

  aErrPacket,		/* 24 */

  aErrConnection,       /* 25 */

  aErrUnknown    /* must be last in list */

} aErr;

#endif /* _aErr_H_ */
