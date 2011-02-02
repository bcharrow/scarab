/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsole_Tests.h					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent Console   */
/*		command handlers.				   */
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

#ifdef aTESTS

#ifndef _aConsole_Tests_H_
#define _aConsole_Tests_H_

#include "aConsole.h"

#define	aNSLOTDEFAULT 		11 /* number of file slots */

aErr aConsole_CreateTests(aConsole* pConsole,
			  aStreamRef testStream);

aErr aConsole_HandleTests(aConsole* pConsole);

aErr aConsole_DestroyTests(aConsole* pConsole);

aBool aConsole_CheckTestExit(aConsole* pConsole,
			     unsigned char module,
		     	     aTEAProcessID pid,
		     	     aVMExit exitCode,
		     	     char* data,
		     	     unsigned char dataLen);

aBool aConsole_CheckTestDisplay(aConsole* pConsole,
			    	unsigned char module,
		     	    	aTEAProcessID pid,
		     	    	char data);

#endif /* _aConsole_Tests_H_ */

#endif /* aTESTS */

