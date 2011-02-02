/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aParseErr.h 						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: tokenizer for the compiler.                        */
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

#ifndef _aParseErr_H_
#define _aParseErr_H_

typedef unsigned short parseErr;

#define prsErrNone			0x00
#define prsErrUnknown			0x01
#define prsErrInvalParam		0x02
#define prsUntermComment		0x03
#define prsUnexpected			0x04
#define prsErr				0x05
#define prsErrExpectAddr		0x06
#define prsErrStrLen			0x07
#define prsErrPktToLong			0x08
#define prsErrEOL			0x09
#define prsErrUntermStr			0x0A
#define prsErrUnknownSym		0x0B
#define prsErrStrExpected		0x0C
#define prsErrIntExpected		0x0D
#define prsErrLblExpected		0x0E
#define prsErrInvalidPreProc		0x0F
#define prsErrIdentExpected		0x10
#define prsErrStringExpected		0x11
#define prsErrFileNotFound		0x12

#define numParseErr			0x20

#endif /* _aParseErr_H_ */
