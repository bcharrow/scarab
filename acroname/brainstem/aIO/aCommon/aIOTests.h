/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aFileTests.h                                              */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Testing module for the aFile shared library.       */
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

#ifndef _aFileTests_H_
#define _aFileTests_H_

#include "aIO.h"

aBool aFileTests(aIOLib ref, aStreamRef outputStream);
aBool aDirectoryTests(aIOLib ref, aStreamRef out);
aBool aStreamTests(aIOLib ref, aStreamRef outputStream);
aBool aZLibFilterTests(aIOLib ioLibRef, aStreamRef out);
aBool aMoreIOTests(aIOLib ioLibRef, aStreamRef out);
aBool aSettingFileTests(aIOLib ioLibRef, aStreamRef out);
aBool aIOTests(aIOLib ref, aStreamRef outputStream);
aBool aTokenizerTests(aIOLib ioLibRef, aStreamRef out);

#endif /* _aFileTests_H_ */
