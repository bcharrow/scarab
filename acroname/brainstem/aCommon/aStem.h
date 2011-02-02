/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStem.h                                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent BrainStem	   */
/*		communication layer.    			   */
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

#ifndef _aStem_H_
#define _aStem_H_

#include "aIO.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * opaque reference to the I/O library
 */

typedef aLIBREF aStemLib;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * define symbol import mechanism
 */

#ifndef aSTEM_EXPORT
#define aSTEM_EXPORT aLIB_IMPORT
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * stem library constants
 */

#define aSTEM_MAGICADDR			173
#define aSTEM_MAXSETTINGLEN		32
#define aSTEM_SETTINGSFILE		"stem.config"
#define aSTEM_TEADELAY_DEFAULT		5
#define	aSTEM_KEY_TEADELAY		"tea-delay"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStem library manipulation routines
 */

#ifdef __cplusplus
extern "C" {
#endif

aSTEM_EXPORT aLIBRETURN 
aStem_GetLibRef(aStemLib* pStemRef,
		aErr* pErr);

aSTEM_EXPORT aLIBRETURN 
aStem_GetNamedLibRef(aStemLib* pStemRef,
		     const char* pName,
		     aErr* pErr);

aSTEM_EXPORT aLIBRETURN 
aStem_ReleaseLibRef(aStemLib stemRef,
		    aErr* pErr);

aSTEM_EXPORT aLIBRETURN 
aStem_GetVersion(aStemLib stemRef,
		 unsigned long *pVersion,
		 aErr* pErr);

#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Maximum number of data bytes in BrainStem packets
 */

#define aSTEMMAXPACKETBYTES  	8


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Opaque packet reference
 */

typedef void* aPacketRef;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * typedef for packet filter
 */

typedef aBool (*aPacketFilter)(const unsigned char module,
			       const unsigned char dataLength,
			       const char* data,
			       void* ref);
typedef aErr (*aHeartbeatCallback)(const aBool bOn,
			           void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * link types
 */

typedef enum {
 kStemModuleStream,
 kStemRelayStream
} aStemStreamType;


#ifdef __cplusplus
extern "C" {
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Stem Routine definitions
 */

aSTEM_EXPORT aLIBRETURN 
aStem_SetStream(aStemLib stemRef, 
		aStreamRef streamRef,
		aStemStreamType streamType,
		aErr* pErr);

aSTEM_EXPORT aLIBRETURN 
aStem_SetHBCallback(aStemLib stemRef,
		    aHeartbeatCallback cbProc,
		    void* cbRef,
		    aErr* pErr);

aSTEM_EXPORT aLIBRETURN 
aStem_DebugLine(aStemLib stemRef,
		const char *line,
		aErr* pErr);

aSTEM_EXPORT aLIBRETURN
aStem_GetPacket(aStemLib stemRef,
		aPacketFilter filterProc,
		void* filterRef,
		unsigned long nMSTimeout,
		aPacketRef* pPacketRef,
		aErr* pErr);

aSTEM_EXPORT aLIBRETURN 
aStem_SendPacket(aStemLib stemRef,
		 const aPacketRef packetRef,
		 aErr* pErr);

aSTEM_EXPORT aLIBRETURN 
aStem_CreateRelayStream(aStemLib stemRef,
			const unsigned char nAddress,
			aStreamRef* pStreamRef,
			aErr* pErr);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Packet Routine definitions
 */

aSTEM_EXPORT aLIBRETURN 
aPacket_Create(aStemLib stemRef,
	       const unsigned char module,
	       const unsigned char length,
	       const char* data,
	       aPacketRef* pPacketRef,
	       aErr* pErr);

aSTEM_EXPORT aLIBRETURN 
aPacket_Format(aStemLib stemRef,
	       const aPacketRef packetRef,
	       char* pBuffer,
	       const unsigned short nMaxLength,
	       aErr* pErr);

aSTEM_EXPORT aLIBRETURN 
aPacket_GetData(aStemLib stemRef,
		const aPacketRef packetRef,
		unsigned char* pModule,
		unsigned char* pLength,
		char* data,
		aErr* pErr);

aSTEM_EXPORT aLIBRETURN 
aPacket_Destroy(aStemLib stemRef,
		const aPacketRef packetRef,
		aErr* pErr);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * TEA Program File I/O definitions
 */

aSTEM_EXPORT aLIBRETURN 
aStem_CreateTEAFileInput(aStemLib stemRef,
			 const unsigned char module,
			 const int nFileNumber,
			 aStreamRef* pStreamRef,
			 aErr* pErr);

aSTEM_EXPORT aLIBRETURN 
aStem_CreateTEAFileOutput(aStemLib stemRef,
			  const unsigned char module,
			  const int nFileNumber,
			  aStreamRef* pStreamRef,
			  aErr* pErr);

#ifdef __cplusplus
}
#endif

#endif /* _aStem_H_ */
