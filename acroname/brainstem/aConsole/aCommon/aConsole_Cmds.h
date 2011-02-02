/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsole_Cmds.h					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent Console	   */
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

#ifndef _aConsole_Cmds_H_
#define _aConsole_Cmds_H_

#include "aConsole.h"

aErr aConsole_HandleExit(aConsole* pConsole,
			 aToken* pFirstToken,
			 aTokenizerRef tokenizer);

aErr aConsole_HandleBatch(aConsole* pConsole,
			  aToken* pFirstToken,
			  aTokenizerRef tokenizer);

aErr aConsole_HandleLaunch(aConsole* pConsole,
			   aToken* pFirstToken,
			   aTokenizerRef tokenizer);

aErr aConsole_HandleAsm(aConsole* pConsole,
			aToken* pFirstToken,
			aTokenizerRef tokenizer);

aErr aConsole_HandleDsm(aConsole* pConsole,
			aToken* pFirstToken,
			aTokenizerRef tokenizer);

#ifdef aTESTS
aErr aConsole_HandleTest(aConsole* pConsole,
			 aToken* pFirstToken,
			 aTokenizerRef tokenizer);
#endif /* aTESTS */

aErr aConsole_HandleLoad(aConsole* pConsole,
			 aToken* pFirstToken,
			 aTokenizerRef tokenizer);

aErr aConsole_HandleUnLoad(aConsole* pConsole,
			   aToken* pFirstToken,
			   aTokenizerRef tokenizer);

aErr aConsole_HandleDebug(aConsole* pConsole,
			  aToken* pFirstToken,
			  aTokenizerRef tokenizer);

aErr aConsole_HandleSP03(aConsole* pConsole,
		         aToken* pFirstToken,
			 aTokenizerRef tokenizer);

aErr aConsole_HandleStat(aConsole* pConsole,
		         aToken* pFirstToken,
			 aTokenizerRef tokenizer);

aErr aConsole_HandleSteep(aConsole* pConsole,
			  int compileFlags,
		          aToken* pFirstToken,
			  aTokenizerRef tokenizer);

aErr aConsole_HandleLeaf(aConsole* pConsole,
			 int compileFlags,
		         aToken* pFirstToken,
			 aTokenizerRef tokenizer);

aErr aConsole_HandleRelay(aConsole* pConsole,
		          aToken* pFirstToken,
			  aTokenizerRef tokenizer);

aErr aConsole_HandleRemote(aConsole* pConsole,
		           aToken* pFirstToken,
			   aTokenizerRef tokenizer);

aErr aConsole_HandleRaw(aConsole* pConsole,
		        aToken* pFirstToken,
			aTokenizerRef tokenizer);

aErr aConsole_HandleSay(aConsole* pConsole,
		        aToken* pFirstToken,
			aTokenizerRef tokenizer);

aErr aConsole_HandleDump(aConsole* pConsole,
		         aToken* pFirstToken,
			 aTokenizerRef tokenizer);

aErr aConsole_HandleHelp(aConsole* pConsole,
		         aToken* pFirstToken,
			 aTokenizerRef tokenizer);

aErr aConsole_HandleMakeDump(aConsole* pConsole, 
			     aToken* pFirstToken, 
			     aTokenizerRef tokenizer);

aErr aConsole_HandleBrainDump(aConsole* pConsole, 
			      aToken* pFirstToken, 
			      aTokenizerRef tokenizer);

#endif /* _aConsole_Cmds_H_ */
