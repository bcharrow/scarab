/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsoleMode_CMUCam.h                                     */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of platform-independent CMUCam Console  */
/*		mode callbacks.					   */
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


#ifndef _aConsoleMode_CMUCam_H_
#define _aConsoleMode_CMUCam_H_

#define DRAWPANEWIDTH      160
#define DRAWPANEHEIGHT     120

#include "aConsole.h"

aErr aCMUCam_RenderOpHTML (
  const unsigned int nParamIndex,
  const unsigned int nBlockIndex,
  aStreamRef reply,
  void* vpRef
);

aErr aConsoleMode_CMUCam_Init(
  aConsole* pConsole
);

aErr aConsoleMode_CMUCam_Cleanup(
  aConsole* pConsole
);

aErr aConsoleMode_CMUCam_Draw(
  aConsole* pConsole
);

aErr aConsoleMode_CMUCam_DrawClick(
  aConsole* pConsole
);

aErr aConsoleMode_CMUCam_HTTP(
  const char* pURL,
  aSymbolTableRef params,
  aStreamRef reply,
  void* vpRef
);

aErr aConsoleMode_CMUCam_Slice(
  aConsole* pConsole
);

aErr aConsoleMode_CMUCam_Line(
  aConsole* pConsole,
  const char* pLine
);

#endif /* _aConsoleMode_CMUCam_H_ */
