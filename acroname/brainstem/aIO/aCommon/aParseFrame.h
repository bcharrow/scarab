/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aParseFrame.h                                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: cross-platform parsing frame definition.           */
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

#ifndef _aParseFrame_H_
#define _aParseFrame_H_

#include "aIO.h"

typedef struct aParseFrame {
  unsigned int		lineNum;
  unsigned int		columnNum;
  unsigned int		prevLineEnd;
  char			frameName[aFILE_NAMEMAXCHARS];
  aStreamRef		input;
  aBool			bBuffered;
  char			buffer;
  aBool			bLineStart;
  struct aParseFrame* 	pSiblings;
  struct aParseFrame*   pChildren;
  struct aParseFrame*   pParent;
} aParseFrame;

aBool aParseFrame_Create(const char* frameName,
			 aStreamRef inputStream,
			 aParseFrame** ppFrame,
			 aErr* pErr);

aBool aParseFrame_AddChild(aParseFrame* pParentFrame,
			   aParseFrame* pChildFrame,
			   aErr* pErr);

aBool aParseFrame_Destroy(aParseFrame* pFrame,
			  aErr* pErr);
			 
#endif /* _aParseFrame_H_ */
