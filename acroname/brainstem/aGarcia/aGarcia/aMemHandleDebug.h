/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aMemHandleDebug.h					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description:	Code that fakes handles for debugging on	   */
/*		platforms that don't implement handles directly.   */
/*		This helps trap errors on environents with rich	   */
/*		debugging environments and avoids finding these    */
/*		on those without source level debugging.	   */
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


#ifndef _aMemHandleDebug_H_
#define _aMemHandleDebug_H_

#ifdef aDEBUG

#include "aOSDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

aMemHandle aMemHandleNew(aMemSize size);
void aMemHandleFree(aMemHandle h);
aMemPtr aMemHandleLock(aMemHandle h);
void aMemHandleUnlock(aMemHandle h);

#ifdef __cplusplus
}
#endif

#endif /* aDEBUG */

#endif /* _aMemHandleDebug_H_ */
