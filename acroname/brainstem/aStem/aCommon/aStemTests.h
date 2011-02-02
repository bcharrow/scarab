/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStemTests.h	                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: stdio testing module for the aStem shared library. */
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

#ifndef _aStemTests_H_
#define _aStemTests_H_

#include "aIO.h"
#include "aStem.h"

aBool aStemTests(aIOLib ioRef, aStreamRef out);
aBool aStemLiveTests(aIOLib ioRef, aStreamRef out);
aBool aStemRelayTests(aIOLib ioRef, aStreamRef out);
aBool aStemTEAFileTestSingle(aIOLib ioRef, aStemLib stemRef,
                        aStreamRef out, unsigned char stemAddr,
                        int nFile, int nFileSize, char* pLabel);
aBool aStemTEAFileTests(aIOLib ioRef, aStemLib stemRef,
                        aStreamRef out, unsigned char stemAddr);

#endif /* _aStemTests_H_ */
