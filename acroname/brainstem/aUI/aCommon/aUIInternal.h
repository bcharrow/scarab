/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aUIInternal.h	                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* descriptUIn: Definition of a platform-independent graphics	   */
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

#ifndef _aUIInternal_H_
#define _aUIInternal_H_

#include "aUI.h"
#include "aIO.h"

#define aUIDialogMSTick	80

#define aUIStructHeader				   	   	   \
  aIOLib		ioRef;					   \
  aMemPoolRef		pGDPPool;				   \
  aStreamRef		buffer;					   \
  int			check

#define aUICHECK	0xF11E

#ifdef aPALM
#define aVALIDUI(p) if (!p) uiErr = aErrParam;
#else /* aPALM */
#define aVALIDUI(p)						   \
  if ((p == NULL) ||						   \
      (((aUI*)p)->check != aUICHECK)) {		   		   \
    uiErr = aErrParam;						   \
  }
#endif /* aPALM */

typedef struct aUI {
  aUIStructHeader;
} aUI;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

aErr aUIInternal_Initialize(aUI* pUI);
aErr aUIInternal_Cleanup(aUI* pUI);

#ifdef __cplusplus 
}
#endif /* __cplusplus */

#endif /* _aUIInternal_H_ */
