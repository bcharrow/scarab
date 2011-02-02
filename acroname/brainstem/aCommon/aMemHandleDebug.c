/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aMemHandleDebug.c                                         */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: implementation of file I/O routines for win32.     */
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

#ifdef aDEBUG

#include "aMemLeakDebug.h"
#include "aMemHandleDebug.h"


typedef struct debugMemHandle {
  int		check;
  int		flags;
  aMemSize	size;
  aMemPtr	data;
} debugMemHandle;


#define debugMemCHECK	0xBEEF
#define debugMemSTALE	0xDEAD

#define memFlagLocked	0x0001

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemHandleNew
 */

aMemHandle aMemHandleNew(aMemSize size)
{
  debugMemHandle* hs;
  hs = (debugMemHandle*)aMemAlloc(sizeof(debugMemHandle));
  if (hs == NULL)
    return(NULL);

  hs->check = debugMemCHECK;
  hs->flags = 0;
  hs->size = size;
  hs->data = (aMemPtr)aMemAlloc((aMemSize)size);
  if (hs->data == NULL) {
    aMemFree(hs);
    return(NULL);
  }

  return((aMemHandle)hs);

} /* aMemHandleNew */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemHandleFree
 */

void aMemHandleFree(aMemHandle h)
{
  debugMemHandle* hs;

  if (h == NULL)
    return;
  hs = (debugMemHandle*)h;
  if (hs->check == debugMemSTALE) {
    aDebugAlert("handle freed more than once");
    return;
  }
  if (hs->check != debugMemCHECK) {
    aDebugAlert("invalid mem handle");
    return;
  }
  hs->check = debugMemSTALE;
  aMemFree(hs->data);
  aMemFree(hs);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemHandleLock
 */

aMemPtr aMemHandleLock(aMemHandle h)
{
  debugMemHandle* hs;

  if (h == NULL)
    return NULL;
  hs = (debugMemHandle*)h;
  if (hs->check == 0xDEAD) {
    aDebugAlert("attempting to lock freed handle");
    return NULL;
  }
  if (hs->check != 0xBEEF) {
    aDebugAlert("attempting to lock invalid handle");
    return NULL;
  }
  if (hs->flags & memFlagLocked) {
    aDebugAlert("attempting to lock locked handle");
    return NULL;
  }
  hs->flags |= memFlagLocked;
  return(hs->data);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMemHandleUnlock
 */

void aMemHandleUnlock(aMemHandle h)
{
  debugMemHandle* hs;
#ifdef aTHRASHMEM
  void *temp;
#endif /* aTHRASHMEM */

  if (h == NULL)
    return;
  hs = (debugMemHandle*)h;
  if (hs->check == 0xDEAD) {
    aDebugAlert("attempting to unlock freed handle");
    return;
  }
  if (hs->check != 0xBEEF) {
    aDebugAlert("attempting to unlock invalid handle");
    return;
  }
  if (!(hs->flags & memFlagLocked)) {
    aDebugAlert("attempting to unlock unlocked handle");
    return;
  }

#ifdef aTHRASHMEM
  /* try to relocate the memory and smash the old memory  
   * before freeing it.
   */
  temp = aMemAlloc((aMemSize)hs->size);
  if (temp != NULL) {
    aMemCopy(temp, hs->data, (aMemSize)hs->size);
    aBZero(hs->data, (aMemSize)hs->size);
    aMemFree(hs->data);
    hs->data = temp;
  }
#endif /* aTHRASHMEM */

  hs->flags &= ~memFlagLocked;
}

#endif /* aDEBUG */
