/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSteepGenOpt.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: implementation of the compiler.                    */
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

#include "aIO.h"
#include "aSteepGen.h"
#include "aSteepGenInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepGen_GetLibRef
 */

aLIBRETURN aSteepGen_GetLibRef(aSteepGenLib* pSteepGenRef,
	 		       aErr* pErr)
{
  aErr sgErr = aSteepGenInternal_Create((aSteepGen**)pSteepGenRef);

  if (pErr != NULL)
    *pErr = sgErr;

  return (aLIBRETURN)(sgErr != aErrNone);

} /* end of aSteepGen_GetLibRef routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepGen_ReleaseLibRef
 */

aLIBRETURN aSteepGen_ReleaseLibRef(aSteepGenLib steepOptRef,
				   aErr* pErr)
{
  aErr sgErr = aSteepGenInternal_Destroy((aSteepGen*)steepOptRef);

  if (pErr != NULL)
    *pErr = sgErr;

  return (aLIBRETURN)(sgErr != aErrNone);

} /* end of aSteepGen_ReleaseLibRef routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepGen_Generate
 */

aLIBRETURN aSteepGen_Generate(aSteepGenLib ref,
			      aSAST* pSAST,
			      aStreamRef result,
			      aErr* pErr)
{
  aErr sgErr = aErrNone;
  aSteepGen* pSteepGen = (aSteepGen*)ref;

  aVALIDSTEEPGEN(pSteepGen);

  if (sgErr == aErrNone)
    sgErr = aSteepGenInternal_Generate(pSteepGen, pSAST, result);

  if (pErr != NULL)
    *pErr = sgErr;

  return (aLIBRETURN)(sgErr != aErrNone);

} /* end of aSteepGen_Generate */
