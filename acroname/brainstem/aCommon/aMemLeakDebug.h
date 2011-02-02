/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aMemLeakDebug.h					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: This file defines a number of common OS-Specific   */
/*              routines.  Since these routines are roughly        */
/*              equivalent but often have different syntax on      */
/*              different platforms, we #define all of them here   */
/*              to allow cross-platform coding.  This also allows  */
/*              us to implement a function missing on a specific   */
/*              platform and allows us to #define away routines    */
/*              that don't make any sense for another platform     */
/*              (such as memory handle locking on Windows or       */
/*              Unix).  Finally, this allows us to add additional  */
/*              checks (and overhead) in debug builds to check for */
/*              memory leaks, etc.                                 */
/*                                                                 */
/*              There is a section for each platform we port to.   */
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

#ifndef _aMemLeakDebug_H_
#define _aMemLeakDebug_H_

#ifdef aLEAKCHECK

#include "aOSDefs.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * defines for memory handling (fixed memory)
 */

#define aMemAlloc(s) aLeakCheckAlloc(s, __FILE__, __LINE__)
#define aMemFree(p) aLeakCheckFree(p, __FILE__, __LINE__)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * actual routine definitions
 */

#ifdef __cplusplus
extern "C" {
#endif

aMemPtr aLeakCheckAlloc(aMemSize size,
			const char* filename,
			int linenum);
void aLeakCheckFree(aMemPtr p,
		    const char* filename,
		    int linenum);
void aLeakCheckCleanup(void);

#ifdef __cplusplus
}
#endif

#else /* aLEAKCHECK */

#define aLeakCheckCleanup()

#endif /* aLEAKCHECK */

#endif /* _aMemLeakDebug_H_ */
