/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aUIInternal.c	                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* descriptUIn: Implementation of a platform-independent graphics  */
/*		layer.						   */
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

#include "aGD.h"
#include "aUIInternal.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUIInternal_Initialize
 */

aErr aUIInternal_Initialize(aUI* pUI)
{
  aErr uiErr = aErrNone;

  if (uiErr == aErrNone)
    aIO_GetLibRef(&pUI->ioRef, &uiErr);

  if (uiErr == aErrNone)
    aMemPool_Create(pUI->ioRef, sizeof(aGDP), 
    		    100, &pUI->pGDPPool, &uiErr);

  /* utility buffer for use by any routine that needs it, is not
   * persistent or thread safe */
  if (uiErr == aErrNone)
    aStreamBuffer_Create(pUI->ioRef, 100, 
    		         &pUI->buffer, &uiErr);

  if (uiErr == aErrNone)
    pUI->check = aUICHECK;

  return uiErr;

} /* aUIInternal_Initialize */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aUIInternal_Cleanup
 */

aErr aUIInternal_Cleanup(aUI* pUI)
{
  aErr uiErr = aErrNone;

  if ((uiErr == aErrNone) && (pUI->buffer))
    aStream_Destroy(pUI->ioRef, pUI->buffer, &uiErr);

  if ((uiErr == aErrNone) && (pUI->pGDPPool))
    aMemPool_Destroy(pUI->ioRef, pUI->pGDPPool, &uiErr);
  
  if (uiErr == aErrNone)
    aIO_ReleaseLibRef(pUI->ioRef, &uiErr);

  return uiErr;

} /* aUIInternal_Cleanup */

