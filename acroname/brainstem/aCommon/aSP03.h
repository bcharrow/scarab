/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSP03.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of cross-platform SP03 Speech module    */
/*              interface routines.				   */
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

#ifndef _aSP03_H_
#define _aSP03_H_

#include "aIO.h"
#include "aStem.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * SP03 defines 
 */

#define aSP03_SERIALCMDLOAD		0x82
#define aSP03_SERIALCMDVERSION		0x83
#define aSP03_MAXSTRINGLEN		81
#define aSP03_CHARTIMEOUT		500
#define aSP03_PHRASETIMEOUT		20000	/* MRW phrases can be long */
#define aSP03_MAXNUMPHRASES		30
#define aSP03_IICADDRESS		0xC4
#define aSP03_IICCOMMAND		0x00
#define aSP03_CMDNOP			0x00
#define aSP03_CMDSPKBUF			0x40
#define aSP03_MAXVOLUME			7
#define aSP03_MAXPITCH			7
#define aSP03_MAXSPEED			3


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * direct commands when the SP03 is directly connected to the 
 * specified stream */

#ifdef __cplusplus
extern "C" {
#endif

aErr aSP03Direct_GetVersion(aStreamRef sp03stream,
		            unsigned char* pSoundChipHardware,
		            unsigned char* pSoundChipSoftware,
		            unsigned char* pModuleFirmware);

aErr aSP03Direct_SpeakPhrase(aStreamRef sp03stream,
		             const unsigned char nPhrase);

aErr aSP03Direct_SpeakString(aStreamRef sp03stream,
		             const unsigned char nVolume,
		             const unsigned char nPitch,
		             const unsigned char nSpeed,
		             const char* pString);

aErr aSP03Direct_LoadPredefines(aStreamRef phraseStream,
				const char* phraseStreamName,
		                aStreamRef sp03Stream,
		                aStreamRef outputStream);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * routed commands for when the SP03 is connected through a 
 * BrainStem routing along the IIC bus. */

aErr aSP03_SpeakPhrase(aStemLib stemLib,
		       const unsigned char nPhrase);

aErr aSP03_SpeakString(aStemLib stemLib,
		       const unsigned char nVolume,
		       const unsigned char nPitch,
		       const unsigned char nSpeed,
		       const char* pString);

#ifdef __cplusplus 
}
#endif

#endif /* _aSP03_H_ */
