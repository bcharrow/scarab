/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsoleTerminal.h			 		   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of platform-independent Terminal mode   */
/*		handling.					   */
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


#ifndef _aConsoleTerminal_H_
#define _aConsoleTerminal_H_

#ifdef aTERMINAL

#include "aConsole.h"

#define aCONSOLETERMCMD "aConsole"

aErr aConsoleTerminal_TimeSlice(aConsole* pConsole);
aErr aConsoleTerminal_HandleLine(aConsole* pConsole,
				 const char* pLine);
aErr aConsoleTerminal_HandleCommand(aConsole* pConsole,
				    aStreamRef commandStream);
aErr aConsoleTerminal_ParseCommand(aConsole* pConsole,
			           aToken* pCommandToken,
			           aTokenizerRef tokenizer);
aErr aConsoleTerminal_XMODEM(aConsole* pConsole,
			     aToken* pFirstToken,
			     aTokenizerRef tokenizer);
aErr aConsoleTerminal_XMODEM_Send(aConsole* pConsole,
			          const char* pCommand,
			          const char* pFilename);

#endif /* aTERMINAL */

#endif /* _aConsoleTerminal_H_ */

