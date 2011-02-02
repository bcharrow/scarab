/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSteep.c						   */
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
#include "aSteep.h"
#include "aSteepInternal.h"
#include "aVersion.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteep_GetVersion
 *
 * version is returned as a 32-bit number.
 *
 */

aLIBRETURN 
aSteep_GetVersion(aSteepLib libRef,
		  unsigned long* pVersion,
		  aErr* pErr)
{
  aErr ioErr = aErrNone;

  if ((libRef == NULL) || (pVersion == NULL))
    ioErr = aErrParam;

  if (ioErr == aErrNone)
    *pVersion = aVERSION_PACK(aVERSION_MAJOR, 
			      aVERSION_MINOR, 
			      aSTEEP_BUILD_NUM); 

  if (pErr != NULL)
    *pErr = ioErr;

  return (aLIBRETURN)(ioErr != aErrNone);

} /* aSteep_GetVersion */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteep_GetLibRef
 */

aSTEEP_EXPORT aLIBRETURN aSteep_GetLibRef(aSteepLib* pSteepRef,
	 				  aErr* pErr)
{
  aErr steepErr = aSteepInternal_Create((aSteep**)pSteepRef);

  if (pErr != NULL)
    *pErr = steepErr;

  return (aLIBRETURN)(steepErr != aErrNone);

} /* end of aSteep_GetLibRef routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteep_ReleaseLibRef
 */

aSTEEP_EXPORT aLIBRETURN aSteep_ReleaseLibRef(aSteepLib steepRef,
																				      aErr* pErr)
{
  aErr steepErr = aSteepInternal_Destroy((aSteep*)steepRef);

  if (pErr != NULL)
    *pErr = steepErr;

  return (aLIBRETURN)(steepErr != aErrNone);

} /* end of aSteep_ReleaseLibRef routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSteep_Compile
 */

aLIBRETURN aSteep_Compile(aSteepLib ref,
			  aStreamRef source,
			  const char* sourceName,
			  int flags,
			  aStreamRef result,
			  aStreamRef out,
			  aStreamRef err, 
			  aErr* pErr)
{
  aErr steepErr = aErrNone;
  aSteep* pSteep = (aSteep*)ref;

  aVALIDSTEEP(pSteep);
  
  if (steepErr == aErrNone)
    steepErr = aSteepInternal_Compile(pSteep,
    				      source,
    				      sourceName,
    				      flags,
    				      result,
    				      out,
    				      err);
  if (pErr != NULL)
    *pErr = steepErr;

  return (aLIBRETURN)(steepErr != aErrNone);

} /* end of aSteep_Compile */

