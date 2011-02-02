/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aUI.c							   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Cross-Platform implementation of basic drawing     */
/*              routines.                                          */
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

#include "aVersion.h"
#include "aUIInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUI_GetVersion
 *
 * version is returned as a 32-bit number.
 *
 */

aLIBRETURN aUI_GetVersion(aUILib ioRef, 
			  unsigned long* pVersion,
			  aErr* pErr)
{
  aErr uiErr = aErrNone;

  if ((ioRef == NULL) || (pVersion == NULL))
    uiErr = aErrParam;

  if (uiErr == aErrNone)
    *pVersion = aVERSION_PACK(aVERSION_MAJOR, 
			      aVERSION_MINOR, 
			      aUI_BUILD_NUM); 

  if (pErr != NULL)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aUI_GetVersion */

