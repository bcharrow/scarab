/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aUI.h                                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: header for I/O library routines for Unix.          */
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

#if defined(aUNIX)

#ifndef aNOX
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#endif /* aNOX */

#include "aUIInternal.h"

typedef struct aUnixUI {
  aUIStructHeader;

#ifndef aNOX
  /* point buffer cache used for type conversion of drawing stuff */
  unsigned int         nPointBufferSize;
  XPoint*              pPointBuffer;

  /* stuff for GD that is one-time or common */
  int                  bXInited;
  XSetWindowAttributes attributes;
  Visual*              pVisual;
  int                  depth;
  int                  screen;
#endif /* aNOX */

  struct aUnixGD*      pGDList;

} aUnixUI;


#endif /* aUNIX */
