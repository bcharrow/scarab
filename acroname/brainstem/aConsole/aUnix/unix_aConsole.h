/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aConsole.h                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Console Program MacX X entry point. 		   */
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

#ifndef _unix_aConsole_H_
#define _unix_aConsole_H_

#include "aConsole.h"

#define kHBColor        0
#define kBGColor        1
#define kAcroColor      2
#define kCommandColor   3
#define kInputColor     4
#define kOutputColor    5
#define kStatusColor    6
#define kMessageColor   7

#define kPairHB         0
#define kPairBar        1
#define kPairCommand    2
#define kPairInput      3
#define kPairOutput     4
#define kPairStatus     5
#define kPairMessage    6

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

typedef struct aUnixConsole {
  aConsoleStructHeader;
  char*	pBlankOutputLine;
  int cur;
  char input[MAXINPUTLINE];
  aBool bHasColor;
} aUnixConsole;

#endif /* _unix_aConsole_H_ */
