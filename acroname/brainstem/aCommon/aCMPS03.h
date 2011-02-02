/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aCMPS03.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of cross-platform CMPS03 Compass module */
/*              interface routines.				   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/*        docs: Documentation can be found in the BrainStem        */
/*		reference under the major topic "C", category      */
/*              aCMPS03                                            */
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

#ifndef _aCMPS03_H_
#define _aCMPS03_H_

#include "aStem.h"


#define aCMPS03_RADIANS		((unsigned char)0)
#define aCMPS03_DEGREES		((unsigned char)1)
#define aCMPS03_ROTATIONS	((unsigned char)2)

#ifdef __cplusplus
extern "C" {
#endif

aErr aCMPS03_GetHeading(aStemLib stemLib,
		        const unsigned char router,
		        const unsigned char units,
		        float* pVal);

#ifdef __cplusplus 
}
#endif

#endif /* _aCMPS03_H_ */
