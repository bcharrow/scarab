/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEAvm.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of cross-platform TEA virtual	   */
/*		machine library.				   */
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

#include "aTEAvm.h"
#include "aTEAvmInternal.h"
#include "aVersion.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvm_GetVersion
 *
 * version is returned as a 32-bit number.
 *
 */

aLIBRETURN aTEAvm_GetVersion(aTEAvmLib TEAvmRef, 
			     unsigned long* pVersion,
			     aErr* pErr)
{
  aErr ioErr = aErrNone;

  if ((TEAvmRef == NULL) || (pVersion == NULL))
    ioErr = aErrParam;

  if (ioErr == aErrNone)
    *pVersion = aVERSION_PACK(aVERSION_MAJOR, 
			      aVERSION_MINOR, 
			      aTEAVM_BUILD_NUM); 

  if (pErr != NULL)
    *pErr = ioErr;

  return (aLIBRETURN)(ioErr != aErrNone);

} /* aTEAvm_GetVersion */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvm_GetLibRef
 */

aLIBRETURN aTEAvm_GetLibRef(aTEAvmLib* pTEAvmRef, 
			    aErr* pErr)
{
  aErr teavmErr = aTEAvmInternal_Create((aTEAvm**)pTEAvmRef);

  if (pErr != NULL)
    *pErr = teavmErr;

  return (teavmErr != aErrNone);
  
} /* aTEAvm_GetLibRef */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvm_ReleaseLibRef
 */

aLIBRETURN aTEAvm_ReleaseLibRef(aTEAvmLib TEAvmRef, 
				aErr* pErr)
{
  aErr teavmErr = aTEAvmInternal_Destroy(TEAvmRef);

  if (pErr != NULL)
    *pErr = teavmErr;

  return (teavmErr != aErrNone);

} /* aTEAvm_ReleaseLibRef */
				

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvm_Initialize
 */

aLIBRETURN aTEAvm_Initialize(aTEAvmLib TEAvmRef,
			     unsigned int nMaxStackSize,
			     unsigned int nMaxProcesses, 
			     aTEAVMIOPortProc portCBProc,
			     void* portCBRef,
			     aErr* pErr)
{
  aErr teavmErr = aTEAvmInternal_Initialize(TEAvmRef,
  					    nMaxStackSize,
  					    nMaxProcesses,
  					    portCBProc,
  					    portCBRef);

  if (pErr != NULL)
    *pErr = teavmErr;

  return (teavmErr != aErrNone);

} /* aTEAvm_Initialize */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvm_TimeSlice
 */

aLIBRETURN aTEAvm_TimeSlice(aTEAvmLib TEAvmRef,
			    aBool* pbProcessed,
			    aErr* pErr)
{
  aErr teavmErr = aTEAvmInternal_TimeSlice(TEAvmRef, pbProcessed);

  if (pErr != NULL)
    *pErr = teavmErr;

  return (teavmErr != aErrNone);

} /* aTEAvm_TimeSlice */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvm_Launch
 */

aLIBRETURN aTEAvm_Launch(aTEAvmLib TEAvmRef,
			 aTEAvmLaunchBlock* pLB,
			 aErr* pErr)
{
  aErr teavmErr = aTEAvmInternal_Launch(TEAvmRef, pLB);

  if (pErr != NULL)
    *pErr = teavmErr;

  return (teavmErr != aErrNone);

} /* aTEAvm_Launch */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvm_Step
 */

aLIBRETURN aTEAvm_Step(aTEAvmLib TEAvmRef, 
		       aTEAvmStepBlock* pSB,
		       aErr* pErr)
{
  aErr teavmErr = aTEAvmInternal_Step(TEAvmRef, pSB);

  if (pErr != NULL)
    *pErr = teavmErr;

  return (teavmErr != aErrNone);

} /* aTEAvm_Step */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvm_ViewStack
 */

aLIBRETURN aTEAvm_ViewStack(aTEAvmLib TEAvmRef, 
			    const aTEAProcessID pid, 
		     	    const unsigned char nMode,
			    const tSTACK nStackOffset,
			    const unsigned char nBytes,
		 	    char* pData,
		       	    aErr* pErr)
{
  aErr teavmErr = aTEAvmInternal_ViewStack(TEAvmRef,
  					   pid,
  					   nMode,
  					   nStackOffset,
  					   nBytes,
  					   pData);

  if (pErr != NULL)
    *pErr = teavmErr;

  return (teavmErr != aErrNone);

} /* aTEAvm_ViewStack */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvm_Shutdown
 */

aLIBRETURN aTEAvm_Kill(aTEAvmLib TEAvmRef, 
		       aTEAProcessID pid, 
		       aErr* pErr)
{
  aErr teavmErr = aTEAvmInternal_Kill(TEAvmRef, 
  				      pid);
  if (pErr != NULL)
    *pErr = teavmErr;

  return (teavmErr != aErrNone);

} /* aTEAvm_Kill */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEAvm_Shutdown
 */

aLIBRETURN aTEAvm_Shutdown(aTEAvmLib TEAvmRef,
			   aErr* pErr)
{
  aErr teavmErr = aTEAvmInternal_Shutdown(TEAvmRef);

  if (pErr != NULL)
    *pErr = teavmErr;

  return (teavmErr != aErrNone);

} /* aTEAvm_Shutdown */

