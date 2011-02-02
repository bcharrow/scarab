/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aBrainDump.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent robotics file */
/*		format utility object.				   */
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

#ifndef _aBrainDump_H_
#define _aBrainDump_H_

#include "aIO.h"
#include "aStem.h"

typedef void* aBDRef;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef aErr (*aBDStatusProc)(const char* pText, const void* ref);

aErr aBD_Create(aIOLib ioRef,
		aStemLib stemRef,
		aBDStatusProc statusProc,
		void* statusRef,
		aBDRef* pBDRef);
aErr aBD_ReadXML(aBDRef bdRef, 
		 aStreamRef xmlStream,
		 aStreamRef status);
aErr aBD_Write(aBDRef bdRef, 
	       aStreamRef destStream);
aErr aBD_Read(aBDRef bdRef,
	      aStreamRef dumpStream);
aErr aBD_CheckLock(aBDRef bdRef,
		   unsigned char module,
		   unsigned int address);
aErr aBD_Dump(aBDRef bdRef);
aErr aBD_Destroy(aBDRef bdRef);

#ifdef __cplusplus 
}
#endif /* __cplusplus */

#endif /* _aBrainDump_H_ */
