/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStemMsg.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Msg codes and pretty printing routine definition.  */
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


#ifndef _aStemMsg_H_
#define _aStemMsg_H_


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * message code definitions
 */

#define iicNoAck		0
#define iicNotWRAddr		1
#define iicQOverflow		2
#define iicQUnderflow		3

#define statePowerReset		4

#define memNoAck		5
#define memRDTooLong		6
#define memAddrRangeErr		7

#define cmdInvalid		8
#define cmdTooLong		9
#define cmdIndexErr		10
#define cmdParamErr		11
#define cmdQOverflow		12
#define cmdQUnderflow		13

#define linkQOverflow		14

#define tmrConflict		15

#define fileInitErr		16
#define fileModeErr		17
#define fileCloseErr		18
#define fileSizeErr		19

#define msgTooLong		20
#define cmdBadRawInput		21

#define iicBOverflow		22
#define iicCMDQOvflw		23
#define iicCMDQUnflw		24

#define vmNotFree		25
#define vmExit			26

#define cmdTooShort		27
#define serialError		28

#define nStemMsg 29	/* must be last plus one */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemMsg_Pretty message formatting routine
 */

#ifdef __cplusplus
extern "C" {
#endif

extern const char* stemMsgText[nStemMsg];

void aStemMsg_Format(unsigned char code, 
		     unsigned char fromAddr,
		     char *pBuffer, 
		     unsigned int nMaxLen);

#ifdef __cplusplus
}
#endif

#endif /* _aStemMsg_H_ */
