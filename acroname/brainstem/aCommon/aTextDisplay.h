/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTextDisplay.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*								   */
/* description: Provides a cross-platform text display object      */
/*              that manages variable sized windows and types of   */
/*              text.                                              */
/*								   */
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

#ifndef _aTextDisplay_H_
#define _aTextDisplay_H_

#include "aIO.h"

typedef void* aTextDisplayRef;
typedef unsigned char aTextDisplayType;

#ifdef __cplusplus
extern "C" {
#endif

aErr aTextDisplay_Create(const aTextDisplayType nVisMask,
			 const unsigned int numLines,
			 const unsigned int width,
			 aTextDisplayRef* ptdRef);

aErr aTextDisplay_Destroy(aTextDisplayRef tdRef);

aErr aTextDisplay_SetWidth(aTextDisplayRef tdRef,
			   const unsigned int width);

aErr aTextDisplay_AddLine(aTextDisplayRef tdRef,
			  const char* line,
			  const aTextDisplayType typeMask);

aErr aTextDisplay_GetDisplayLines(aTextDisplayRef tdRef,
			          unsigned int* pNumLines,
			          const aTextDisplayType typeMask);

aErr aTextDisplay_PrepareEnum(aTextDisplayRef tdRef,
			      const unsigned int nStartLine,
			      const aTextDisplayType typeMask);

aErr aTextDisplay_NextEnum(aTextDisplayRef tdRef,
			   aStreamRef buffer,
			   const unsigned int maxLen,
			   aTextDisplayType* pTypeMask);

aErr aTextDisplay_GetLine(aTextDisplayRef tdRef,
			  const unsigned int nIndex,
			  const aTextDisplayType typeMask,
			  aStreamRef buffer);

#ifdef __cplusplus
}
#endif

#endif /* _aTextDisplay_H_ */
