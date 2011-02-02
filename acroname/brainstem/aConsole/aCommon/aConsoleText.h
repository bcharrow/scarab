/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsoleText.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Isolated printable text that is embedded in the    */
/*		code. Makes for easier internationalization.	   */
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

#ifndef _aConsoleText_H_
#define _aConsoleText_H_

#define aT_PROD_NAME_TXT		"BrainStem Console"

#define aT_LICENSE_FILE			"license.dat"
#define aT_LICENSE_MSG			"licensed to: "
#define aT_EVALUATION_MSG		"this software is not licensed"

#define aT_ACTION_MENU_TXT		"Process"
#define aT_QUIT_MENU_TXT		"Quit"
#define aT_COMPILE_MENU_TXT		"Compile..."
#define aT_RUN_MENU_TXT			"Run..."

#define aT_WELCOME_TXT1			"Welcome to the Acroname"
#define aT_WELCOME_TXT2			aT_PROD_NAME_TXT
#define aT_WELCOME_TXT3			"Version: "aT_PROD_VERS_TXT

#define aT_HOST_NAME			"host"

#define aT_CMDLINENAME			"command line"

#define aSTATUS_STREAM_SET		"stream set"
#define aSTATUS_NO_SERIAL		"serial error"
#define aSTATUS_NO_LINK			"link not set"
#define aSTATUS_SP03_MODE		"running in SP03 direct mode"
#define aSTATUS_BRAINSTEM_MODE		"running in BrainStem mode"
#define aSTATUS_TERMINAL_MODE		"running in terminal mode"
#define aSTATUS_CMUCAM_MODE		"running in CMUCam mode"
#define aSTATUS_LINKTYPE                "linktype: "
#define aSTATUS_PORTNAME                "portname: "
#define aSTATUS_BAUDRATE                "baudrate: %d"
#define aSTATUS_USBID			"usb_id: %d "

#ifdef aTESTS
#define aSTATUS_TEST_STARTING		"starting tests"
#define aSTATUS_TEST_FINISHED		"test finished"
#define aSTATUS_TEST_FAILED		"can't test: "
#define aSTATUS_TEST_BADMODE		"unknown test mode"
#define aSTATUS_TEST_DEBUGMODE		"debug mode"
#define aSTATUS_TEST_RUNMODE		"run mode"
#define aSTATUS_TEST_TARGETHOST		"targeting host"
#define aSTATUS_TEST_TARGETMODULE	"targeting module "
#define aSTATUS_TEST_TARGETBAD		"bad target"
#define aSTATUS_TEST_CONCUR		"concurrency is "
#define aSTATUS_TEST_BADCONCUR		"illegal concurrency"
#define aSTATUS_TEST_DBGFAILURE		"debug step failure"
#define aSTATUS_TEST_PACKETINPUT	"packet input mode"
#define aSTATUS_TEST_STRINGINPUT	"string input mode"
#define aSTATUS_TEST_BADINPUT		"unknown input mode"
#define aSTATUS_TEST_BADSLOT		"invalid number of slots"
#define aSTATUS_TEST_BADPROCESS		"invalid number of processes"
#define aSTATUS_TEST_USINGSLOTS		"number of slots is "
#define aSTATUS_TEST_USINGPROCESSES	"number of processes is "
#endif /* aTESTS */

#define aCMD_ILLEGAL_INPUT		"illegal input"
#define aCMD_ILLEGAL_CMD		"illegal command"
#define aCMD_UNEXPECTED_PARAM		"unexpected parameter"
#define aCMD_EXPECTED_STRING		"string expected"
#define aCMD_EXPECTED_NUMBER		"number expected"
#define aCMD_INVALID_MODULE		"invalid module number"
#define aCMD_MISSING_PARAM		"missing parameter"
#define aCMD_FILE_NOT_FOUND		"file not found"
#define aCMD_NO_MODULE_FOUND		"no module found"
#define aCMD_MODULE_DOES_NOT_REPLY	"module does not reply"
#define aCMD_FILE_ERROR			"unknown file error"
#define aCMD_FILE_IO_ERROR		"file IO or access error"
#define aCMD_FILE_INVALID		"invalid file"
#define aCMD_SLOT_INVALID		"invalid slot"
#define aCMD_BATCH_START		"starting batch:"
#define aCMD_BATCH_END			"batch finished:"
#define aCMD_MISSING_FILENAME		"missing filename"
#define aCMD_MISSING_MODULE		"missing module ref"
#define aCMD_SLOTINIT_FAILED		"unable to access module slot"
#define aCMD_LOAD_FAILED		"file load failed"
#define aCMD_UNLOAD_FAILED		"file unload failed"
#define aCMD_LOAD_SUCCEEDED		"file loaded"
#define aCMD_UNLOAD_SUCCEEDED		"file unloaded"
#define aCMD_BAD_INET_ADDR		"bad ip address"
#define	aCMD_NO_PROGRAM_SPECIFIED	"no program specified"
#define	aCMD_HELP_CMD_EXPECTED		"command expected"
#define aCMD_SPEECH_RANGE_ERROR		"phrase is out of range"
#define aCMD_SPEECH_DONE		"speaking"
#define aCMD_SPEECH_MISSING		"speech module not found"
#define aCMD_SPEECH_ERROR		"speech error"
#define aCMD_INVALID_MODE		"invalid in this mode"
#define aCMD_MISSING_OPTION		"missing command option"
#define aCMD_INVALID_OPTION		"invalid command option"
#define aCMD_VALUE_RANGE		"settings out of range, using %d"
#define aCMD_COMPILE_SIZE		"compiled into %d bytes"

#define aSP03_VOLUME			"volume (0 loud - 7 quiet) = %d"
#define aSP03_PITCH			"pitch (0 high - 7 low) = %d"
#define aSP03_SPEED			"speed (0 slow - 3 fast) = %d"

#define aVM_PROCESS_EXIT		"vm exited"
#define aVM_PROCESS_LAUNCH		"vm launch"

#define aPKT_ILLEGAL_ADDR		"illegal IIC address"
#define aPKT_ILLEGAL_LEN		"packet too large"

#define aRLY_STARTING			"relaying on "

#define aDEBUG_UNSETTING_STREAM		"Unsetting link stream for non-brainstem mode"

#endif /* _aConsoleText_H_ */
