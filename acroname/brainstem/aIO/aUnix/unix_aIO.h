/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aIO.h                                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: header for I/O library routines for unix.          */
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

#ifdef aUNIX

#ifndef _unix_aIO_H_
#define _unix_aIO_H_

#include "aOSDefs.h"
#include "aIOInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Local unix specific socket server structure.
 */

typedef struct aUNIXIOSocket {
  unsigned int			address;
  unsigned short		port;
  int				socket;
  unsigned short		count;
  struct aUNIXIOSocket* 	pNext;
  void*				vpUnixIOLib;
} aUNIXIOSocket;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Local windows specific file lib structure.  This is hidden by the 
 * opaque reference type aFileLib.
 */

typedef struct aUnixIOLib {
  aIOStructHeader;
  aUNIXIOSocket*	 	pServerPortList;
} aUnixIOLib;

#endif /* _unix_aIO_H_ */

#endif /* aUNIX */
