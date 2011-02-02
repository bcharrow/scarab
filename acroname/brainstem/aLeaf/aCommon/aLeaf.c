/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aLeaf.c						   */
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
#include "aLeaf.h"
#include "aLeafInternal.h"
#include "aVersion.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLeaf_GetVersion
 *
 * version is returned as a 32-bit number.
 *
 */

aLIBRETURN aLeaf_GetVersion(aLeafLib libRef, 
			    unsigned long* pVersion,
			    aErr* pErr)
{
  aErr leafErr = aErrNone;

  if ((libRef == NULL) || (pVersion == NULL))
    leafErr = aErrParam;

  if (leafErr == aErrNone)
    *pVersion = aVERSION_PACK(aVERSION_MAJOR, 
			      aVERSION_MINOR, 
			      aLEAF_BUILD_NUM); 

  if (pErr != NULL)
    *pErr = leafErr;

  return (aLIBRETURN)(leafErr != aErrNone);

} /* aLeaf_GetVersion */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLeaf_GetLibRef
 */

aLEAF_EXPORT aLIBRETURN aLeaf_GetLibRef(aLeafLib* pLeafRef,
	 				  aErr* pErr)
{
  aErr leafErr = aLeafInternal_Create((aLeaf**)pLeafRef);

  if (pErr != NULL)
    *pErr = leafErr;

  return (aLIBRETURN)(leafErr != aErrNone);

} /* end of aLeaf_GetLibRef routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLeaf_ReleaseLibRef
 */

aLEAF_EXPORT aLIBRETURN aLeaf_ReleaseLibRef(aLeafLib leafRef,
																				      aErr* pErr)
{
  aErr leafErr = aLeafInternal_Destroy((aLeaf*)leafRef);

  if (pErr != NULL)
    *pErr = leafErr;

  return (aLIBRETURN)(leafErr != aErrNone);

} /* end of aLeaf_ReleaseLibRef routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aLeaf_Compile
 */

aLIBRETURN aLeaf_Compile(aLeafLib ref,
			 aStreamRef source,
			 const char* sourceName,
			 int flags,
			 aStreamRef result,
			 aStreamRef out,
			 aStreamRef err, 
			 aErr* pErr)
{
  aErr leafErr = aErrNone;
  aLeaf* pLeaf = (aLeaf*)ref;

  aVALIDLEAF(pLeaf);
  
  if (leafErr == aErrNone)
    leafErr = aLeafInternal_Compile(pLeaf,
    				    source,
    				    sourceName,
    				    flags,
    				    result,
    				    out,
    				    err);
  if (pErr != NULL)
    *pErr = leafErr;

  return (aLIBRETURN)(leafErr != aErrNone);

} /* end of aLeaf_Compile */
