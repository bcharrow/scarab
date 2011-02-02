/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aPad.h                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Routine definitions for accessing the scratch pad  */
/*              on the BrainStem.				   */
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

#ifndef _aPad_H_
#define _aPad_H_

#include "aStem.h"

#ifdef __cplusplus
extern "C" {
#endif

aErr aPad_ReadChar(aStemLib stemRef,
			  const unsigned char module,
			  const unsigned char padIndex,
			  char* pVal);

aErr aPad_ReadInt(aStemLib stemRef,
			const unsigned char module,
			const unsigned char padIndex,
			int* pVal);

aErr aPad_WriteChar(aStemLib stemRef,
			 const unsigned char module,
			 const unsigned char padIndex,
			 const char val);

aErr aPad_WriteInt(aStemLib stemRef,
			 const unsigned char module,
			 const unsigned char padIndex,
			 const int val);

#ifdef __cplusplus 
}
#endif

#endif /* aPad_H_ */
