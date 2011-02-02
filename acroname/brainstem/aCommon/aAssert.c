/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aAssert.c                                                 */
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

/* fools compilers that don't want an empty file */
int aAssertFile;

#ifdef aDEBUG

#include "aOSDefs.h"
#include "aAssert.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAssertion
 */

void aAssertion(
  const char* expression,
  const char* filename,
  int linenum
)
{
  char msg[1000];
  char num[20];

  aStringCopySafe(msg, 1000, "Assertion Failed: \"");
  aStringCatSafe(msg, 1000, expression);
  aStringCatSafe(msg, 1000, "\" line: ");
  aStringFromInt(num, linenum);
  aStringCatSafe(msg, 1000, num);
  aStringCatSafe(msg, 1000, " file: ");
  aStringCatSafe(msg, 1000, filename);
  aDebugAlert(msg);
  
} /* aAssertion */

#endif /* aDEBUG */

