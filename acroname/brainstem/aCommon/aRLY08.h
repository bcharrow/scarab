/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aRLY08.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of cross-platform RLY08 Compass module */
/*              interface routines.				   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/*        docs: Documentation can be found in the BrainStem        */
/*		reference under the major topic "C", category      */
/*              aRLY08                                            */
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

#ifndef _aRLY08_H_
#define _aRLY08_H_

#include "aStem.h"
#include "aModuleUtil.h"
#include "aRLY08.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local defines
 */

#define aRLY08_ADDRESS			0x70
#define aRLY08_GETDATATIMEOUTMS	100
#define aRLY08_I2CSETTING 0 /* Stem cmdVal for I2C of 100kHz */ 

#ifdef __cplusplus
extern "C" {
#endif

aErr aRLY08_RelayOn(aStemLib stemLib,
		        const unsigned char router,
						const unsigned char rly08address,
		        const unsigned char relay);

aErr aRLY08_RelayOff(aStemLib stemLib,
		        const unsigned char router,
						const unsigned char rly08address,
		        const unsigned char relay);
						
aErr aRLY08_RelaysAllOn(aStemLib stemLib,
										const unsigned char router,
										const unsigned char rly08address);			
										
aErr aRLY08_RelaysAllOff(aStemLib stemLib,
										const unsigned char router,
										const unsigned char rly08address);																		

#ifdef __cplusplus 
}
#endif

#endif /* _aRLY08_H_ */
