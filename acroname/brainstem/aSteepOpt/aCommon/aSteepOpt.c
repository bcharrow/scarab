/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSteepOptOpt.c						   */
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
#include "aSteepOpt.h"
#include "aSteepOptInternal.h"
#include "aVersion.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepOpt_GetLibRef
 */

aLIBRETURN aSteepOpt_GetLibRef(aSteepOptLib* pSteepOptRef,
	 		       aErr* pErr)
{
  aErr soErr = aSteepOptInternal_Create((aSteepOpt**)pSteepOptRef);

  if (pErr != NULL)
    *pErr = soErr;

  return (aLIBRETURN)(soErr != aErrNone);

} /* end of aSteepOpt_GetLibRef routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepOpt_ReleaseLibRef
 */

aLIBRETURN aSteepOpt_ReleaseLibRef(aSteepOptLib steepOptRef,
				   aErr* pErr)
{
  aErr soErr = aSteepOptInternal_Destroy((aSteepOpt*)steepOptRef);

  if (pErr != NULL)
    *pErr = soErr;

  return (aLIBRETURN)(soErr != aErrNone);

} /* end of aSteepOpt_ReleaseLibRef routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteepOpt_Optimize
 */

aLIBRETURN aSteepOpt_Optimize(aSteepOptLib ref,
			      aSAST* pSAST,
			      aErr* pErr)
{
  aErr soErr = aErrNone;
  aSteepOpt* pSteepOpt = (aSteepOpt*)ref;

  aVALIDSTEEPOPT(pSteepOpt);

  if (soErr == aErrNone)
    soErr = aSteepOptInternal_Optimize(pSteepOpt, pSAST);

  if (pErr != NULL)
    *pErr = soErr;

  return (aLIBRETURN)(soErr != aErrNone);

} /* end of aSteepOpt_Optimize */
