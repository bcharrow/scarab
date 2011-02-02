/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aIR.h                                                     */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: BrainStem GP 2.0 IR routines.                      */
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

#ifndef _aIR_H_
#define _aIR_H_

#include "aStem.h"
#include "aStemCore.h"

#define aIR_PROTOCOL_NEC 0
#define aIR_PROTOCOL_RC5 1

#define aIR_TXMASK 0x40
#define aIR_RXMASK 0x39

#ifdef __cplusplus
extern "C" {
#endif

aErr aIR_ConfigRX(const aStemLib stemRef,
		     const unsigned char module,
		     char irConfiguration);

aErr aIR_ConfigTX(const aStemLib stemRef,
		     const unsigned char module,
		     char irConfiguration);
		     
aErr aIR_RXInt(const aStemLib stemRef,
		     const unsigned char module,
		     int* pValue);

aErr aIR_TXInt(const aStemLib stemRef,
		     const unsigned char module,
		     int irValue);

#ifdef __cplusplus
}
#endif

#endif /* _aIR_H_ */
